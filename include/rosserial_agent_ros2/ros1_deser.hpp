#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include <stdexcept>
#include <vector>
#include <type_traits>

namespace ros1
{

    // Cursor over a byte buffer (little-endian).
    class Cursor
    {
    public:
        Cursor(const uint8_t *data, size_t len) : p_(data), end_(data + len) {}

        bool remaining(size_t n) const { return (size_t)(end_ - p_) >= n; }
        size_t remaining() const { return (size_t)(end_ - p_); }

        uint8_t u8()
        {
            require(1);
            return *p_++;
        }

        int8_t i8()
        {
            require(1);
            return static_cast<int8_t>(*p_++);
        }

        uint16_t u16()
        {
            require(2);
            uint16_t v = (uint16_t)p_[0] | ((uint16_t)p_[1] << 8);
            p_ += 2;
            return v;
        }

        uint32_t u32()
        {
            require(4);
            uint32_t v = (uint32_t)p_[0] | ((uint32_t)p_[1] << 8) | ((uint32_t)p_[2] << 16) | ((uint32_t)p_[3] << 24);
            p_ += 4;
            return v;
        }

        int32_t i32() { return (int32_t)u32(); }

        float f32()
        {
            require(4);
            float out;
            uint32_t v = u32_peek();
            std::memcpy(&out, &v, sizeof(float));
            p_ += 4;
            return out;
        }

        std::string str()
        {
            uint32_t n = u32();
            require(n);
            const char *s = (const char *)p_;
            p_ += n;
            return std::string(s, s + n);
        }

        // Read primitive array with ROS 1 format: uint32 length + elements
        template <typename T>
        std::vector<T> array_primitive()
        {
            static_assert(std::is_same<T, uint8_t>::value ||
                              std::is_same<T, int8_t>::value ||
                              std::is_same<T, float>::value,
                          "unsupported primitive");
            uint32_t n = u32();
            std::vector<T> v;
            v.reserve(n);
            if constexpr (std::is_same<T, uint8_t>::value)
            {
                require(n);
                v.insert(v.end(), p_, p_ + n);
                p_ += n;
            }
            else if constexpr (std::is_same<T, int8_t>::value)
            {
                require(n);
                for (uint32_t i = 0; i < n; ++i)
                    v.push_back(static_cast<int8_t>(*p_++));
            }
            else if constexpr (std::is_same<T, float>::value)
            {
                require((size_t)n * 4);
                for (uint32_t i = 0; i < n; ++i)
                    v.push_back(f32());
            }
            return v;
        }

    private:
        uint32_t u32_peek() const
        {
            uint32_t v = (uint32_t)p_[0] | ((uint32_t)p_[1] << 8) | ((uint32_t)p_[2] << 16) | ((uint32_t)p_[3] << 24);
            return v;
        }
        void require(size_t n) const
        {
            if (!remaining(n))
                throw std::runtime_error("ros1::Cursor out of data");
        }

        const uint8_t *p_;
        const uint8_t *end_;
    };

} // namespace ros1
