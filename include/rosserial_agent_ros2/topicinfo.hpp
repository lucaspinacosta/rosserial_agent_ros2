#pragma once
#include <cstdint>
#include <string>
#include "rosserial_agent_ros2/ros1_deser.hpp"

// Wire schema for rosserial_msgs/TopicInfo (ROS 1):
// uint16 topic_id
// string topic_name
// string message_type
// string md5sum
// int32  buffer_size
//
// Special endpoint IDs (for reference):
// 0 PUBLISHER, 1 SUBSCRIBER, 2 SERVICE_SERVER, 4 SERVICE_CLIENT,
// 6 PARAMETER_REQUEST, 7 LOG, 10 TIME, 11 TX_STOP
struct TopicInfoMsg
{
    uint16_t topic_id;
    std::string topic_name;
    std::string message_type;
    std::string md5sum;
    int32_t buffer_size;

    static TopicInfoMsg parse(const uint8_t *data, size_t len)
    {
        ros1::Cursor c(data, len);
        TopicInfoMsg m;
        m.topic_id = c.u16();
        m.topic_name = c.str();
        m.message_type = c.str();
        m.md5sum = c.str();
        m.buffer_size = c.i32();
        return m;
    }
};
