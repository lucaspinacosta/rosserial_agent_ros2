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

#include <unordered_map>
#include <vector>
#include <memory>

using std::placeholders::_1;

static constexpr uint16_t TOPIC_ID_TOPICINFO = 0; // rosserial reserved system topic id

// Simple holder for the per-topic ROS 2 publisher we create dynamically
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
    const int buf_bytes = node->declare_parameter<int>("read_buffer_bytes", 2048);

    RCLCPP_INFO(node->get_logger(), "Opening %s @ %d baud", dev.c_str(), baud);

    SerialPort sp;
    if (!sp.open(dev, baud))
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to open serial device %s", dev.c_str());
        rclcpp::shutdown();
        return 1;
    }

    RosserialParser parser;
    TopicMap topics;

    // Per-topic publisher handles keyed by endpoint id
    std::unordered_map<uint16_t, PubHandles> pubs;

    std::vector<uint8_t> buf(buf_bytes);
    rclcpp::WallRate idle_rate(std::chrono::milliseconds(2));

    while (rclcpp::ok())
    {
        auto n = sp.read(buf.data(), buf.size());
        if (n > 0)
        {
            auto frames = parser.ingest(buf.data(), static_cast<size_t>(n));
            for (auto &f : frames)
            {
                if (f.topic_id == TOPIC_ID_TOPICINFO)
                {
                    try
                    {
                        auto ti = TopicInfoMsg::parse(f.payload.data(), f.payload.size());
                        TopicInfoRecord rec{ti.topic_id, ti.topic_name, ti.message_type, ti.md5sum, ti.buffer_size};
                        topics.upsert(rec);

                        // Create ROS 2 publisher lazily for supported inbound types
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
                            RCLCPP_INFO(node->get_logger(), "Prepared ROS2 *subscriber* for %s (%s) [outbound path will be added next]",
                                        rec.topic_name.c_str(), rec.message_type.c_str());
                            // For inbound publish we don't use it, but we log the presence.
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

                // Data frame: decode based on known type and publish
                auto info = topics.by_id(f.topic_id);
                if (!info)
                {
                    RCLCPP_DEBUG(node->get_logger(), "Data for unknown id=%u (%zu bytes)", (unsigned)f.topic_id, f.payload.size());
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
                        // Convert layout
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
                        {
                            it->second.f32ma->publish(msg);
                        }
                        else
                        {
                            // Create on the fly if missing (shouldn't happen if TopicInfo arrived)
                            auto pub = node->create_publisher<std_msgs::msg::Float32MultiArray>(info->topic_name, rclcpp::SensorDataQoS());
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
                        {
                            it->second.u8->publish(msg);
                        }
                        else
                        {
                            auto pub = node->create_publisher<std_msgs::msg::UInt8>(info->topic_name, rclcpp::QoS(10).reliable());
                            pubs[f.topic_id].u8 = pub;
                            pub->publish(msg);
                        }
                    }
                    else if (info->message_type == "std_msgs/Int8MultiArray")
                    {
                        // Depending on direction, MCU might *subscribe* here; if it publishes too, we can handle inbound:
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
                        {
                            it->second.i8ma->publish(msg);
                        }
                        else
                        {
                            auto pub = node->create_publisher<std_msgs::msg::Int8MultiArray>(info->topic_name, rclcpp::QoS(10).reliable());
                            pubs[f.topic_id].i8ma = pub;
                            pub->publish(msg);
                        }
                    }
                    else
                    {
                        // Unsupported type (yet): just log
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
            if (!parser.last_error().empty())
            {
                RCLCPP_DEBUG(node->get_logger(), "parser note: %s", parser.last_error().c_str());
            }
            idle_rate.sleep();
        }
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
