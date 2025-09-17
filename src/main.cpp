#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "rosserial_agent_ros2/serial_port.hpp"
#include "rosserial_agent_ros2/rosserial_parser.hpp"
#include "rosserial_agent_ros2/ros1_deser.hpp"
#include "rosserial_agent_ros2/ros1_multiarray.hpp"
#include "rosserial_agent_ros2/topicinfo.hpp"
#include "rosserial_agent_ros2/topic_map.hpp"
#include "rosserial_agent_ros2/rosserial_frame.hpp"

#include <unordered_map>
#include <vector>
#include <memory>
#include <chrono>
#include <sstream>
#include <iomanip>

static constexpr uint16_t TOPIC_ID_TOPICINFO = 0; // rosserial reserved system topic id

// per-topic ROS 2 publishers
struct PubHandles
{
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr f32ma;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr u8;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr i8ma;
};

// ---- helpers --------------------------------------------------------------
static std::string hex(const uint8_t *p, size_t n)
{
    std::ostringstream os;
    for (size_t i = 0; i < n; ++i)
    {
        os << std::uppercase << std::hex << std::setfill('0') << std::setw(2) << int(p[i]);
        if (i + 1 < n)
            os << ' ';
    }
    return os.str();
}

struct StaticTopicSpec
{
    uint16_t id;
    std::string name;
    std::string type;
};

// Parse "id=126,name=/joints,type=std_msgs/Float32MultiArray"
static bool parse_static_spec(const std::string &s, StaticTopicSpec &out)
{
    uint16_t id = 0;
    std::string name, type;
    std::istringstream is(s);
    std::string kv;
    auto trim = [](std::string &x)
    {
        while (!x.empty() && isspace(x.front()))
            x.erase(x.begin());
        while (!x.empty() && isspace(x.back()))
            x.pop_back();
    };
    while (std::getline(is, kv, ','))
    {
        auto pos = kv.find('=');
        if (pos == std::string::npos)
            continue;
        auto k = kv.substr(0, pos), v = kv.substr(pos + 1);
        trim(k);
        trim(v);
        if (k == "id")
            id = static_cast<uint16_t>(std::stoi(v));
        else if (k == "name")
            name = v;
        else if (k == "type")
            type = v;
    }
    if (name.empty() || type.empty())
        return false;
    out = {id, name, type};
    return true;
}
// --------------------------------------------------------------------------

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rosserial_agent");

    const std::string dev = node->declare_parameter<std::string>("dev", "/dev/ttyACM0");
    const int baud = node->declare_parameter<int>("baud", 115200);
    const int buf_bytes = node->declare_parameter<int>("read_buffer_bytes", 2048);

    // debug/compat params
    const bool reset_on_open = node->declare_parameter<bool>("reset_on_open", true);
    const int boot_quiet_ms = node->declare_parameter<int>("boot_quiet_ms", 1200);
    const bool dump_rx = node->declare_parameter<bool>("dump_rx", false);
    const bool dump_tx = node->declare_parameter<bool>("dump_tx", false);

    const bool sensor_reliable = node->declare_parameter<bool>("sensor_reliable", false);

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
        RCLCPP_INFO(node->get_logger(), "Pulsing DTR/RTS to reset board...");
        sp.pulse_dtr(50, 0);
        sp.pulse_rts(50, 0);
    }
    if (boot_quiet_ms > 0)
    {
        RCLCPP_INFO(node->get_logger(), "Waiting %d ms for boot...", boot_quiet_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(boot_quiet_ms));
    }

    RosserialParser parser;
    TopicMap topics;
    std::unordered_map<uint16_t, PubHandles> pubs;

    // === Static topic mappings (optional) ===
    std::vector<std::string> static_entries =
        node->declare_parameter<std::vector<std::string>>("static_topics", {});
    if (!static_entries.empty())
    {
        RCLCPP_INFO(node->get_logger(), "Loading %zu static topic mappings...", static_entries.size());
        for (const auto &s : static_entries)
        {
            StaticTopicSpec spec{};
            if (!parse_static_spec(s, spec))
            {
                RCLCPP_WARN(node->get_logger(), "Bad static_topics entry: '%s' (use id=..,name=..,type=..)", s.c_str());
                continue;
            }
            TopicInfoRecord rec{spec.id, spec.name, spec.type, "", 512};
            topics.upsert(rec);
            auto &h = pubs[rec.endpoint_id];
            if (rec.message_type == "std_msgs/Float32MultiArray" && !h.f32ma)
            {
                auto qos = sensor_reliable ? rclcpp::QoS(10).reliable()
                                           : rclcpp::SensorDataQoS(); // best_effort
                h.f32ma = node->create_publisher<std_msgs::msg::Float32MultiArray>(rec.topic_name, qos);
                RCLCPP_INFO(node->get_logger(), "Static map: publisher for %s (%s) on id=%u (reliable=%s)",
                            rec.topic_name.c_str(), rec.message_type.c_str(), (unsigned)rec.endpoint_id,
                            sensor_reliable ? "true" : "false");
            }
            else if (rec.message_type == "std_msgs/UInt8" && !h.u8)
            {
                auto qos = sensor_reliable ? rclcpp::QoS(10).reliable()
                                           : rclcpp::QoS(10).best_effort();
                h.u8 = node->create_publisher<std_msgs::msg::UInt8>(rec.topic_name, qos);
                RCLCPP_INFO(node->get_logger(), "Static map: publisher for %s (%s) on id=%u (reliable=%s)",
                            rec.topic_name.c_str(), rec.message_type.c_str(), (unsigned)rec.endpoint_id,
                            sensor_reliable ? "true" : "false");
            }
            else if (rec.message_type == "std_msgs/Int8MultiArray" && !h.i8ma)
            {
                auto qos = sensor_reliable ? rclcpp::QoS(10).reliable()
                                           : rclcpp::QoS(10).best_effort();
                h.i8ma = node->create_publisher<std_msgs::msg::Int8MultiArray>(rec.topic_name, qos);
                RCLCPP_INFO(node->get_logger(), "Static map: publisher for %s (%s) on id=%u (reliable=%s)",
                            rec.topic_name.c_str(), rec.message_type.c_str(), (unsigned)rec.endpoint_id,
                            sensor_reliable ? "true" : "false");
            }
            else
            {
                RCLCPP_INFO(node->get_logger(), "Static map: id=%u name='%s' type='%s' (no decoder yet)",
                            (unsigned)rec.endpoint_id, rec.topic_name.c_str(), rec.message_type.c_str());
            }
        }
    }

    // passive sniff (optional, helps see boot banners)
    std::vector<uint8_t> buf(buf_bytes);
    auto sniff_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < sniff_until)
    {
        auto n = sp.read(buf.data(), buf.size());
        if (n > 0 && dump_rx)
            RCLCPP_INFO(node->get_logger(), "RX(s) %zd: %s", n, hex(buf.data(), (size_t)n).c_str());
    }

    // === Handshake: RequestTopics, FE + FF variants ===
    bool any_topicinfo_seen = false;
    auto last_request = std::chrono::steady_clock::now();
    int burst_requests_left = 3;

    auto send_request_topics_both = [&]()
    {
        const std::vector<uint8_t> empty;
        auto f_fe = rosserial_wire::build_frame(0, empty, 0xFE);
        auto n1 = sp.write(f_fe.data(), f_fe.size());
        if (dump_tx)
            RCLCPP_INFO(node->get_logger(), "TX FE: %s", hex(f_fe.data(), f_fe.size()).c_str());
        auto f_ff = rosserial_wire::build_frame(0, empty, 0xFF);
        auto n2 = sp.write(f_ff.data(), f_ff.size());
        if (dump_tx)
            RCLCPP_INFO(node->get_logger(), "TX FF: %s", hex(f_ff.data(), f_ff.size()).c_str());
        RCLCPP_INFO(node->get_logger(), "Sent RequestTopics FE(%zd) + FF(%zd)", n1, n2);
    };

    send_request_topics_both();
    last_request = std::chrono::steady_clock::now();

    rclcpp::WallRate idle_rate(std::chrono::milliseconds(2));

    while (rclcpp::ok())
    {
        // quick retry burst
        if (!any_topicinfo_seen && burst_requests_left > 0 &&
            std::chrono::steady_clock::now() - last_request > std::chrono::milliseconds(200))
        {
            send_request_topics_both();
            last_request = std::chrono::steady_clock::now();
            --burst_requests_left;
        }
        // slow retry
        if (!any_topicinfo_seen &&
            std::chrono::steady_clock::now() - last_request > std::chrono::seconds(1))
        {
            send_request_topics_both();
            last_request = std::chrono::steady_clock::now();
        }

        auto n = sp.read(buf.data(), buf.size());
        if (n > 0)
        {
            if (dump_rx)
                RCLCPP_INFO(node->get_logger(), "RX %zd: %s", n, hex(buf.data(), (size_t)n).c_str());
            auto frames = parser.ingest(buf.data(), static_cast<size_t>(n));
            for (auto &f : frames)
            {
                if (f.topic_id == TOPIC_ID_TOPICINFO)
                {
                    any_topicinfo_seen = true;
                    last_request = std::chrono::steady_clock::now();
                    try
                    {
                        auto ti = TopicInfoMsg::parse(f.payload.data(), f.payload.size());
                        TopicInfoRecord rec{ti.topic_id, ti.topic_name, ti.message_type, ti.md5sum, ti.buffer_size};
                        topics.upsert(rec);

                        auto &handles = pubs[rec.endpoint_id];
                        if (rec.message_type == "std_msgs/Float32MultiArray" && !handles.f32ma)
                        {
                            handles.f32ma = node->create_publisher<std_msgs::msg::Float32MultiArray>(rec.topic_name, rclcpp::SensorDataQoS());
                            RCLCPP_INFO(node->get_logger(), "Created ROS2 publisher for %s (%s)", rec.topic_name.c_str(), rec.message_type.c_str());
                        }
                        else if (rec.message_type == "std_msgs/UInt8" && !handles.u8)
                        {
                            handles.u8 = node->create_publisher<std_msgs::msg::UInt8>(rec.topic_name, rclcpp::QoS(10).reliable());
                            RCLCPP_INFO(node->get_logger(), "Created ROS2 publisher for %s (%s)", rec.topic_name.c_str(), rec.message_type.c_str());
                        }
                        else if (rec.message_type == "std_msgs/Int8MultiArray" && !handles.i8ma)
                        {
                            handles.i8ma = node->create_publisher<std_msgs::msg::Int8MultiArray>(rec.topic_name, rclcpp::QoS(10).reliable());
                            RCLCPP_INFO(node->get_logger(), "Prepared ROS2 *subscriber* for %s (%s)", rec.topic_name.c_str(), rec.message_type.c_str());
                        }
                        else
                        {
                            RCLCPP_INFO(node->get_logger(), "TopicInfo: id=%u name='%s' type='%s' md5=%s buf=%d",
                                        (unsigned)rec.endpoint_id, rec.topic_name.c_str(),
                                        rec.message_type.c_str(), rec.md5sum.c_str(), rec.buffer_size);
                        }
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_WARN(node->get_logger(), "Failed to parse TopicInfo: %s", e.what());
                    }
                    continue;
                }

                // Data frame → publish via known mapping (static or negotiated)
                auto info = topics.by_id(f.topic_id);
                if (!info)
                {
                    RCLCPP_DEBUG(node->get_logger(), "Data for unknown id=%u (%zu bytes)",
                                 (unsigned)f.topic_id, f.payload.size());
                    continue;
                }

                try
                {
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

                        auto it = pubs.find(f.topic_id);
                        if (it != pubs.end() && it->second.f32ma)
                            it->second.f32ma->publish(msg);
                        else
                        {
                            auto qos = sensor_reliable ? rclcpp::QoS(10).reliable()
                                                       : rclcpp::SensorDataQoS();
                            auto pub = node->create_publisher<std_msgs::msg::Float32MultiArray>(info->topic_name, qos);
                            pubs[f.topic_id].f32ma = pub;
                            pub->publish(msg);
                        }
                    }
                    else if (info->message_type == "std_msgs/UInt8")
                    {
                        ros1::Cursor c(f.payload.data(), f.payload.size());
                        std_msgs::msg::UInt8 msg;
                        msg.data = c.u8();

                        auto it = pubs.find(f.topic_id);
                        if (it != pubs.end() && it->second.u8)
                            it->second.u8->publish(msg);
                        else
                        {
                            auto qos = sensor_reliable ? rclcpp::QoS(10).reliable()
                                                       : rclcpp::QoS(10).best_effort();
                            auto pub = node->create_publisher<std_msgs::msg::UInt8>(info->topic_name, qos);
                            pubs[f.topic_id].u8 = pub;
                            pub->publish(msg);
                        }
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

                        auto it = pubs.find(f.topic_id);
                        if (it != pubs.end() && it->second.i8ma)
                            it->second.i8ma->publish(msg);
                        else
                        {
                            auto qos = sensor_reliable ? rclcpp::QoS(10).reliable()
                                                       : rclcpp::QoS(10).best_effort();
                            auto pub = node->create_publisher<std_msgs::msg::Int8MultiArray>(info->topic_name, qos);
                            pubs[f.topic_id].i8ma = pub;
                            pub->publish(msg);
                        }
                    }
                    else
                    {
                        RCLCPP_DEBUG(node->get_logger(),
                                     "Data: id=%u name='%s' type='%s' payload=%zu bytes (decoder missing)",
                                     (unsigned)f.topic_id, info->topic_name.c_str(), info->message_type.c_str(), f.payload.size());
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(node->get_logger(),
                                "Decode error for id=%u name='%s' type='%s': %s",
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
