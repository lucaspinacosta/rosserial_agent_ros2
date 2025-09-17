#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "rosserial_agent_ros2/serial_port.hpp"
#include "rosserial_agent_ros2/ros1_deser.hpp"
#include "rosserial_agent_ros2/ros1_multiarray.hpp"
#include "rosserial_agent_ros2/topicinfo.hpp"
#include "rosserial_agent_ros2/topic_map.hpp"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <cctype>
#include <algorithm>

static constexpr uint16_t TOPIC_ID_TOPICINFO = 0;
static constexpr size_t LOG_ELEMS_MAX = 32;

// ================= helper utils =================
namespace wire
{
    struct Frame
    {
        uint16_t topic_id{0};
        std::vector<uint8_t> payload;
    };
    inline uint8_t chk(uint32_t sum) { return uint8_t(0xFFu - (sum & 0xFFu)); }
}

static std::string ascii_preview(const uint8_t *p, size_t n, size_t max_len = 48)
{
    std::ostringstream os;
    os << "'";
    size_t m = std::min(n, max_len);
    for (size_t i = 0; i < m; ++i)
    {
        unsigned char c = p[i];
        os << (std::isprint(c) ? char(c) : '.');
    }
    if (n > m)
        os << "...";
    os << "'";
    return os.str();
}

static inline std::string normalize_topic(std::string s)
{
    if (s.empty())
        return s;
    if (s[0] != '/')
        s.insert(s.begin(), '/');
    return s;
}

static std::string format_layout(const std_msgs::msg::MultiArrayLayout &L)
{
    std::ostringstream os;
    os << "{dim:[";
    for (size_t i = 0; i < L.dim.size(); ++i)
    {
        if (i)
            os << ", ";
        os << "{label:'" << L.dim[i].label << "', size:" << L.dim[i].size
           << ", stride:" << L.dim[i].stride << "}";
    }
    os << "], offset:" << L.data_offset << "}";
    return os.str();
}
static std::string format_vec_f32(const std::vector<float> &v)
{
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os << std::setprecision(6);
    os << "[";
    size_t n = std::min(LOG_ELEMS_MAX, v.size());
    for (size_t i = 0; i < n; ++i)
    {
        if (i)
            os << ", ";
        os << v[i];
    }
    if (v.size() > n)
        os << ", ... (" << v.size() << " total)";
    os << "]";
    return os.str();
}
static std::string format_vec_i8(const std::vector<int8_t> &v)
{
    std::ostringstream os;
    os << "[";
    size_t n = std::min(LOG_ELEMS_MAX, v.size());
    for (size_t i = 0; i < n; ++i)
    {
        if (i)
            os << ", ";
        os << int(v[i]);
    }
    if (v.size() > n)
        os << ", ... (" << v.size() << " total)";
    os << "]";
    return os.str();
}

// ================= strict wire parser =================
class StrictParser
{
public:
    explicit StrictParser(rclcpp::Logger logger, bool verbose = false)
        : logger_(logger), verbose_(verbose) { buf_.reserve(2048); }

    std::vector<wire::Frame> ingest(const uint8_t *data, size_t n)
    {
        std::vector<wire::Frame> out;
        buf_.insert(buf_.end(), data, data + n);

        while (true)
        {
            if (buf_.size() < 8)
                break;

            size_t i = 0;
            while (i + 1 < buf_.size() && !(buf_[i] == 0xFF && (buf_[i + 1] == 0xFE || buf_[i + 1] == 0xFF)))
                ++i;
            if (i > 0)
            {
                if (verbose_)
                    RCLCPP_DEBUG(logger_, "PARSER: dropped %zu pre-sync bytes", i);
                buf_.erase(buf_.begin(), buf_.begin() + i);
                if (buf_.size() < 8)
                    break;
            }

            if (buf_.size() < 5)
                break;
            uint8_t len_l = buf_[2], len_h = buf_[3], len_chk = buf_[4];
            uint8_t want_len_chk = wire::chk(uint32_t(len_l) + uint32_t(len_h));
            if (len_chk != want_len_chk)
            {
                buf_.erase(buf_.begin());
                continue;
            }

            uint16_t len = uint16_t(len_l) | (uint16_t(len_h) << 8);
            const size_t total = 2 + 3 + 2 + len + 1;
            if (buf_.size() < total)
                break;

            uint8_t topic_l = buf_[5], topic_h = buf_[6];
            uint16_t topic_id = uint16_t(topic_l) | (uint16_t(topic_h) << 8);

            const uint8_t *payload = &buf_[7];
            uint8_t got_payload_chk = buf_[7 + len];

            uint32_t sum = uint32_t(topic_l) + uint32_t(topic_h);
            for (size_t k = 0; k < len; ++k)
                sum += uint32_t(payload[k]);
            uint8_t want_payload_chk = wire::chk(sum);
            if (got_payload_chk != want_payload_chk)
            {
                buf_.erase(buf_.begin());
                continue;
            }

            wire::Frame f;
            f.topic_id = topic_id;
            f.payload.assign(payload, payload + len);
            out.emplace_back(std::move(f));

            buf_.erase(buf_.begin(), buf_.begin() + total);
        }
        return out;
    }

private:
    rclcpp::Logger logger_;
    bool verbose_;
    std::vector<uint8_t> buf_;
};

// ================= static topic spec =================
struct StaticTopicSpec
{
    bool id_auto{false};
    uint16_t id{0};
    std::string name; // "/joints"
    std::string type; // "std_msgs/Float32MultiArray"
};
static bool parse_spec(const std::string &s, StaticTopicSpec &out)
{
    std::istringstream is(s);
    std::string kv;
    auto trim = [](std::string &x)
    {
        while (!x.empty() && isspace(x.front()))
            x.erase(x.begin());
        while (!x.empty() && isspace(x.back()))
            x.pop_back();
    };
    bool have_name = false, have_type = false, have_id = false;
    uint16_t id = 0;
    bool id_auto = false;
    std::string name, type;
    while (std::getline(is, kv, ','))
    {
        auto p = kv.find('=');
        if (p == std::string::npos)
            continue;
        auto k = kv.substr(0, p), v = kv.substr(p + 1);
        trim(k);
        trim(v);
        if (k == "id")
        {
            if (v == "auto" || v == "AUTO" || v == "Auto")
            {
                id_auto = true;
                have_id = true;
            }
            else
            {
                id = static_cast<uint16_t>(std::stoi(v));
                have_id = true;
            }
        }
        else if (k == "name")
        {
            name = v;
            have_name = true;
        }
        else if (k == "type")
        {
            type = v;
            have_type = true;
        }
    }
    if (!have_name || !have_type)
        return false;
    if (!have_id)
        id_auto = true;
    out = {id_auto, id, normalize_topic(name), type};
    return true;
}

// ================= main =================
struct PubHandles
{
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr f32ma;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr u8;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr i8ma;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rosserial_agent");

    const std::string dev = node->declare_parameter<std::string>("dev", "/dev/ttyACM0");
    const int baud = node->declare_parameter<int>("baud", 115200);
    const int buf_n = node->declare_parameter<int>("read_buffer_bytes", 2048);

    const bool reset_on_open = node->declare_parameter<bool>("reset_on_open", true);
    const int boot_quiet_ms = node->declare_parameter<int>("boot_quiet_ms", 1200);
    const bool dump_rx = node->declare_parameter<bool>("dump_rx", true);
    const bool dump_tx = node->declare_parameter<bool>("dump_tx", false);
    const bool parser_verbose = node->declare_parameter<bool>("parser_verbose", false);

    const bool sensor_reliable = node->declare_parameter<bool>("sensor_reliable", true);

    std::vector<std::string> specs =
        node->declare_parameter<std::vector<std::string>>("static_topics", {});

    RCLCPP_INFO(node->get_logger(), "Opening %s @ %d baud", dev.c_str(), baud);
    SerialPort sp;
    if (!sp.open(dev, baud))
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to open serial device %s", dev.c_str());
        rclcpp::shutdown();
        return 1;
    }
    if (reset_on_open)
    {
        RCLCPP_INFO(node->get_logger(), "Pulsing DTR/RTS...");
        sp.pulse_dtr(50, 0);
        sp.pulse_rts(50, 0);
    }
    if (boot_quiet_ms > 0)
    {
        RCLCPP_INFO(node->get_logger(), "Waiting %d ms for boot...", boot_quiet_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(boot_quiet_ms));
    }

    TopicMap topics;                               // id -> (name,type,md5,buf)
    std::unordered_map<uint16_t, PubHandles> pubs; // id -> pubs

    struct Key
    {
        std::string name;
        std::string type;
        bool operator==(Key const &o) const { return name == o.name && type == o.type; }
    };
    struct KeyHash
    {
        size_t operator()(Key const &k) const
        {
            return std::hash<std::string>()(k.name) ^ (std::hash<std::string>()(k.type) << 1);
        }
    };
    std::unordered_set<Key, KeyHash> pending;

    auto qos_sensor = sensor_reliable ? rclcpp::QoS(10).reliable() : rclcpp::SensorDataQoS();

    // Seed static mappings (normalized names)
    if (!specs.empty())
        RCLCPP_INFO(node->get_logger(), "Static topics: %zu", specs.size());
    for (auto &s : specs)
    {
        StaticTopicSpec st{};
        if (!parse_spec(s, st))
        {
            RCLCPP_WARN(node->get_logger(), "Bad static_topics entry: '%s'", s.c_str());
            continue;
        }
        if (st.id_auto)
        {
            pending.insert(Key{st.name, st.type});
            RCLCPP_INFO(node->get_logger(), "Waiting to bind '%s' (%s) to an id...", st.name.c_str(), st.type.c_str());
        }
        else
        {
            TopicInfoRecord rec{st.id, st.name, st.type, "", 512};
            topics.upsert(rec);
            auto &h = pubs[st.id];
            if (st.type == "std_msgs/Float32MultiArray" && !h.f32ma)
            {
                h.f32ma = node->create_publisher<std_msgs::msg::Float32MultiArray>(st.name, qos_sensor);
            }
            else if (st.type == "std_msgs/UInt8" && !h.u8)
            {
                h.u8 = node->create_publisher<std_msgs::msg::UInt8>(st.name, rclcpp::QoS(10).reliable());
            }
            else if (st.type == "std_msgs/Int8MultiArray" && !h.i8ma)
            {
                h.i8ma = node->create_publisher<std_msgs::msg::Int8MultiArray>(st.name, rclcpp::QoS(10).reliable());
            }
            RCLCPP_DEBUG(node->get_logger(), "Bound fixed id=%u -> %s (%s)", (unsigned)st.id, st.name.c_str(), st.type.c_str());
        }
    }

    // Strict parser
    StrictParser parser(node->get_logger(), parser_verbose);

    // Optional sniff
    std::vector<uint8_t> buf(buf_n);
    auto sniff_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < sniff_until)
    {
        auto n = sp.read(buf.data(), buf.size());
        if (n > 0 && dump_rx)
        {
            RCLCPP_DEBUG(node->get_logger(), "RX(s): %zd bytes preview=%s", n, ascii_preview(buf.data(), (size_t)n).c_str());
        }
    }

    // Minimal RequestTopics (FE + FF)
    auto send_request_topics_both = [&]()
    {
        auto build = [](uint8_t flag)
        {
            std::vector<uint8_t> v;
            v.reserve(8);
            v.push_back(0xFF);
            v.push_back(flag);
            v.push_back(0x00);
            v.push_back(0x00);
            v.push_back(wire::chk(0)); // len + len_chk
            v.push_back(0x00);
            v.push_back(0x00);         // topic 0
            v.push_back(wire::chk(0)); // payload chk
            return v;
        };
        auto f_fe = build(0xFE), f_ff = build(0xFF);
        auto n1 = sp.write(f_fe.data(), f_fe.size());
        auto n2 = sp.write(f_ff.data(), f_ff.size());
        if (dump_tx)
        {
            RCLCPP_DEBUG(node->get_logger(), "TX FE len=%zu", f_fe.size());
            RCLCPP_DEBUG(node->get_logger(), "TX FF len=%zu", f_ff.size());
        }
        RCLCPP_DEBUG(node->get_logger(), "Sent RequestTopics FE(%zd) + FF(%zd)", n1, n2);
    };
    send_request_topics_both();

    rclcpp::WallRate idle_rate(std::chrono::milliseconds(2));
    auto last_req = std::chrono::steady_clock::now();
    int burst = 2;

    auto bind_and_make_pubs = [&](uint16_t id, const std::string &name, const std::string &type)
    {
        const std::string norm_name = normalize_topic(name);
        TopicInfoRecord rec{id, norm_name, type, "", 512};
        topics.upsert(rec);
        auto &h = pubs[id];
        if (type == "std_msgs/Float32MultiArray" && !h.f32ma)
        {
            h.f32ma = node->create_publisher<std_msgs::msg::Float32MultiArray>(norm_name, qos_sensor);
        }
        else if (type == "std_msgs/UInt8" && !h.u8)
        {
            h.u8 = node->create_publisher<std_msgs::msg::UInt8>(norm_name, rclcpp::QoS(10).reliable());
        }
        else if (type == "std_msgs/Int8MultiArray" && !h.i8ma)
        {
            h.i8ma = node->create_publisher<std_msgs::msg::Int8MultiArray>(norm_name, rclcpp::QoS(10).reliable());
        }
        RCLCPP_DEBUG(node->get_logger(), "Bound id=%u -> %s (%s)", (unsigned)id, norm_name.c_str(), type.c_str());
    };

    // Heuristic binder if no TopicInfo
    auto try_bind_unknown = [&](uint16_t id, const std::vector<uint8_t> &payload) -> bool
    {
        if (!pending.empty() && payload.size() == 1)
        {
            for (auto it = pending.begin(); it != pending.end(); ++it)
            {
                if (it->type == "std_msgs/UInt8")
                {
                    std::string name = it->name;
                    pending.erase(it);
                    bind_and_make_pubs(id, name, "std_msgs/UInt8");
                    return true;
                }
            }
        }
        try
        { // Float32MultiArray
            ros1::Cursor c(payload.data(), payload.size());
            auto layout = ros1::parse_layout(c);
            auto data = c.array_primitive<float>();
            if (!c.remaining() && layout.dim.size() <= 8 && data.size() <= 1024)
            {
                for (auto it = pending.begin(); it != pending.end(); ++it)
                {
                    if (it->type == "std_msgs/Float32MultiArray")
                    {
                        std::string name = it->name;
                        pending.erase(it);
                        bind_and_make_pubs(id, name, "std_msgs/Float32MultiArray");
                        return true;
                    }
                }
            }
        }
        catch (...)
        {
        }
        try
        { // Int8MultiArray
            ros1::Cursor c(payload.data(), payload.size());
            auto layout = ros1::parse_layout(c);
            auto data = c.array_primitive<int8_t>();
            if (!c.remaining() && layout.dim.size() <= 8 && data.size() <= 4096)
            {
                for (auto it = pending.begin(); it != pending.end(); ++it)
                {
                    if (it->type == "std_msgs/Int8MultiArray")
                    {
                        std::string name = it->name;
                        pending.erase(it);
                        bind_and_make_pubs(id, name, "std_msgs/Int8MultiArray");
                        return true;
                    }
                }
            }
        }
        catch (...)
        {
        }
        return false;
    };

    while (rclcpp::ok())
    {
        auto now = std::chrono::steady_clock::now();
        if (now - last_req > std::chrono::milliseconds(burst > 0 ? 250 : 1000))
        {
            send_request_topics_both();
            last_req = now;
            if (burst > 0)
                --burst;
        }

        auto n = sp.read(buf.data(), buf.size());
        if (n > 0)
        {
            if (dump_rx)
                RCLCPP_DEBUG(node->get_logger(), "RX %zd bytes preview=%s", n, ascii_preview(buf.data(), (size_t)n).c_str());

            // auto frames = StrictParser(node->get_logger(), parser_verbose).ingest(buf.data(), (size_t)n); // local stateless for this read
            // NOTE: if you prefer cumulative parsing, keep one StrictParser instance and reuse it:
            auto frames = parser.ingest(buf.data(), (size_t)n);

            RCLCPP_DEBUG(node->get_logger(), "Parsed %zu frame(s)", frames.size());

            for (auto &f : frames)
            {
                if (f.topic_id == TOPIC_ID_TOPICINFO)
                {
                    try
                    {
                        auto ti = TopicInfoMsg::parse(f.payload.data(), f.payload.size());
                        const std::string norm_name = normalize_topic(ti.topic_name);
                        TopicInfoRecord rec{ti.topic_id, norm_name, ti.message_type, ti.md5sum, ti.buffer_size};
                        topics.upsert(rec);

                        // Always create publishers for inbound types (even if not pending)
                        bind_and_make_pubs(rec.endpoint_id, rec.topic_name, rec.message_type);
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_WARN(node->get_logger(), "TopicInfo parse error: %s", e.what());
                    }
                    continue;
                }

                // Resolve topic_id
                auto info = topics.by_id(f.topic_id);
                if (!info)
                {
                    if (!pending.empty() && try_bind_unknown(f.topic_id, f.payload))
                    {
                        info = topics.by_id(f.topic_id);
                    }
                }
                if (!info)
                {
                    RCLCPP_INFO(node->get_logger(), "Data for unknown id=%u (%zu bytes); no bind yet", (unsigned)f.topic_id, f.payload.size());
                    continue;
                }

                // -------- Decode & publish (with lazy pub creation) --------
                try
                {
                    const std::string topic = normalize_topic(info->topic_name);
                    if (info->message_type == "std_msgs/Float32MultiArray")
                    {
                        ros1::Cursor c(f.payload.data(), f.payload.size());
                        auto layout = ros1::parse_layout(c);
                        auto data = c.array_primitive<float>();

                        std_msgs::msg::Float32MultiArray msg;
                        msg.layout.dim.resize(layout.dim.size());
                        for (size_t i = 0; i < layout.dim.size(); ++i)
                        {
                            msg.layout.dim[i].label = layout.dim[i].label;
                            msg.layout.dim[i].size = layout.dim[i].size;
                            msg.layout.dim[i].stride = layout.dim[i].stride;
                        }
                        msg.layout.data_offset = layout.data_offset;
                        msg.data = std::move(data);

                        RCLCPP_DEBUG(node->get_logger(), "FRAME id=%u len=%zu", (unsigned)f.topic_id, f.payload.size());
                        RCLCPP_DEBUG(node->get_logger(), "PUB id=%u topic=%s type=%s layout=%s data=%s",
                                    (unsigned)f.topic_id, topic.c_str(), info->message_type.c_str(),
                                    format_layout(msg.layout).c_str(), format_vec_f32(msg.data).c_str());

                        auto &h = pubs[f.topic_id];
                        if (!h.f32ma)
                        {
                            RCLCPP_INFO(node->get_logger(), "Creating publisher on-demand for %s (%s)", topic.c_str(), info->message_type.c_str());
                            h.f32ma = node->create_publisher<std_msgs::msg::Float32MultiArray>(topic, sensor_reliable ? rclcpp::QoS(10).reliable() : rclcpp::SensorDataQoS());
                        }
                        h.f32ma->publish(msg);
                    }
                    else if (info->message_type == "std_msgs/UInt8")
                    {
                        ros1::Cursor c(f.payload.data(), f.payload.size());
                        std_msgs::msg::UInt8 msg;
                        msg.data = c.u8();

                        RCLCPP_DEBUG(node->get_logger(), "FRAME id=%u len=%zu", (unsigned)f.topic_id, f.payload.size());
                        RCLCPP_DEBUG(node->get_logger(), "PUB id=%u topic=%s type=%s value=%u",
                                    (unsigned)f.topic_id, topic.c_str(), info->message_type.c_str(), (unsigned)msg.data);

                        auto &h = pubs[f.topic_id];
                        if (!h.u8)
                        {
                            RCLCPP_INFO(node->get_logger(), "Creating publisher on-demand for %s (%s)", topic.c_str(), info->message_type.c_str());
                            h.u8 = node->create_publisher<std_msgs::msg::UInt8>(topic, rclcpp::QoS(10).reliable());
                        }
                        h.u8->publish(msg);
                    }
                    else if (info->message_type == "std_msgs/Int8MultiArray")
                    {
                        ros1::Cursor c(f.payload.data(), f.payload.size());
                        auto layout = ros1::parse_layout(c);
                        auto data = c.array_primitive<int8_t>();

                        std_msgs::msg::Int8MultiArray msg;
                        msg.layout.dim.resize(layout.dim.size());
                        for (size_t i = 0; i < layout.dim.size(); ++i)
                        {
                            msg.layout.dim[i].label = layout.dim[i].label;
                            msg.layout.dim[i].size = layout.dim[i].size;
                            msg.layout.dim[i].stride = layout.dim[i].stride;
                        }
                        msg.layout.data_offset = layout.data_offset;
                        msg.data = std::move(data);

                        RCLCPP_DEBUG(node->get_logger(), "FRAME id=%u len=%zu", (unsigned)f.topic_id, f.payload.size());
                        RCLCPP_DEBUG(node->get_logger(), "PUB id=%u topic=%s type=%s layout=%s data=%s",
                                    (unsigned)f.topic_id, topic.c_str(), info->message_type.c_str(),
                                    format_layout(msg.layout).c_str(), format_vec_i8(msg.data).c_str());

                        auto &h = pubs[f.topic_id];
                        if (!h.i8ma)
                        {
                            RCLCPP_INFO(node->get_logger(), "Creating publisher on-demand for %s (%s)", topic.c_str(), info->message_type.c_str());
                            h.i8ma = node->create_publisher<std_msgs::msg::Int8MultiArray>(topic, rclcpp::QoS(10).reliable());
                        }
                        h.i8ma->publish(msg);
                    }
                    else
                    {
                        RCLCPP_INFO(node->get_logger(), "No decoder for %s (id=%u)", info->message_type.c_str(), (unsigned)f.topic_id);
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(node->get_logger(), "Decode error id=%u name='%s' type='%s': %s",
                                (unsigned)f.topic_id, info->topic_name.c_str(), info->message_type.c_str(), e.what());
                }
            }
        }
        else
        {
            idle_rate.sleep();
        }

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
