#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <cstring>
#include <type_traits>

namespace ros1
{

    // Append POD little-endian
    inline void a_u8(std::vector<uint8_t> &out, uint8_t v) { out.push_back(v); }
    inline void a_u16(std::vector<uint8_t> &out, uint16_t v)
    {
        out.push_back(uint8_t(v & 0xFF));
        out.push_back(uint8_t((v >> 8) & 0xFF));
    }
    inline void a_u32(std::vector<uint8_t> &out, uint32_t v)
    {
        out.push_back(uint8_t(v & 0xFF));
        out.push_back(uint8_t((v >> 8) & 0xFF));
        out.push_back(uint8_t((v >> 16) & 0xFF));
        out.push_back(uint8_t((v >> 24) & 0xFF));
    }
    inline void a_i32(std::vector<uint8_t> &out, int32_t v) { a_u32(out, static_cast<uint32_t>(v)); }

    inline void a_f32(std::vector<uint8_t> &out, float f)
    {
        static_assert(sizeof(float) == 4, "");
        uint32_t v;
        std::memcpy(&v, &f, sizeof(float));
        a_u32(out, v);
    }

    // ROS 1 string: uint32 length then bytes (no NUL)
    inline void a_str(std::vector<uint8_t> &out, const std::string &s)
    {
        a_u32(out, static_cast<uint32_t>(s.size()));
        out.insert(out.end(), s.begin(), s.end());
    }

    // ROS 1 primitive arrays: uint32 length + elements
    template <typename T>
    inline void a_array(std::vector<uint8_t> &out, const std::vector<T> &v)
    {
        static_assert(std::is_same<T, uint8_t>::value ||
                          std::is_same<T, int8_t>::value ||
                          std::is_same<T, float>::value,
                      "unsupported primitive array");
        a_u32(out, static_cast<uint32_t>(v.size()));
        if constexpr (std::is_same<T, uint8_t>::value)
        {
            out.insert(out.end(), v.begin(), v.end());
        }
        else if constexpr (std::is_same<T, int8_t>::value)
        {
            for (auto x : v)
                out.push_back(static_cast<uint8_t>(x));
        }
        else if constexpr (std::is_same<T, float>::value)
        {
            for (auto x : v)
                a_f32(out, x);
        }
    }

} // namespace ros1
