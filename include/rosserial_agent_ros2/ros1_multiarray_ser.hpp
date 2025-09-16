#pragma once
#include <vector>
#include <string>
#include "rosserial_agent_ros2/ros1_ser.hpp"

namespace ros1
{

    // Serialize std_msgs/MultiArrayLayout
    struct MultiArrayDimensionS
    {
        std::string label;
        uint32_t size;
        uint32_t stride;
    };
    struct MultiArrayLayoutS
    {
        std::vector<MultiArrayDimensionS> dim;
        uint32_t data_offset{0};
    };

    inline void serialize_layout(std::vector<uint8_t> &out, const MultiArrayLayoutS &L)
    {
        a_u32(out, static_cast<uint32_t>(L.dim.size()));
        for (auto &d : L.dim)
        {
            a_str(out, d.label);
            a_u32(out, d.size);
            a_u32(out, d.stride);
        }
        a_u32(out, L.data_offset);
    }

} // namespace ros1
