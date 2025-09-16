#pragma once
#include <cstdint>
#include <vector>

namespace rosserial_wire
{

    // checksum helper (8-bit 1's complement)
    inline uint8_t chk(uint32_t sum_mod256) { return static_cast<uint8_t>(0xFFu - (sum_mod256 & 0xFFu)); }

    // Build a full rosserial frame for a given topic id and payload
    inline std::vector<uint8_t> build_frame(uint16_t topic_id, const std::vector<uint8_t> &payload)
    {
        const uint8_t SYNC1 = 0xFF, SYNC2 = 0xFE;
        uint16_t len = static_cast<uint16_t>(payload.size());
        uint8_t len_l = uint8_t(len & 0xFF), len_h = uint8_t((len >> 8) & 0xFF);
        uint8_t len_chk = chk(uint32_t(len_l) + uint32_t(len_h));
        uint8_t topic_l = uint8_t(topic_id & 0xFF), topic_h = uint8_t((topic_id >> 8) & 0xFF);

        std::vector<uint8_t> out;
        out.reserve(2 + 3 + 2 + payload.size() + 1);
        out.push_back(SYNC1);
        out.push_back(SYNC2);
        out.push_back(len_l);
        out.push_back(len_h);
        out.push_back(len_chk);
        out.push_back(topic_l);
        out.push_back(topic_h);
        out.insert(out.end(), payload.begin(), payload.end());
        uint32_t s = uint32_t(topic_l) + uint32_t(topic_h);
        for (auto b : payload)
            s += uint32_t(b);
        out.push_back(chk(s));
        return out;
    }

} // namespace rosserial_wire
