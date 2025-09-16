#pragma once
#include <string>
#include <vector>
#include "rosserial_agent_ros2/ros1_deser.hpp"

// Minimal parser for std_msgs/MultiArrayLayout and *MultiArray messages (ROS 1 wire)
namespace ros1
{

    struct MultiArrayDimension
    {
        std::string label;
        uint32_t size;
        uint32_t stride;
    };

    struct MultiArrayLayout
    {
        std::vector<MultiArrayDimension> dim;
        uint32_t data_offset;
    };

    inline MultiArrayLayout parse_layout(Cursor &c)
    {
        MultiArrayLayout L;
        uint32_t n = c.u32(); // number of dimensions
        L.dim.reserve(n);
        for (uint32_t i = 0; i < n; ++i)
        {
            MultiArrayDimension d;
            d.label = c.str();
            d.size = c.u32();
            d.stride = c.u32();
            L.dim.push_back(std::move(d));
        }
        L.data_offset = c.u32();
        return L;
    }

} // namespace ros1
