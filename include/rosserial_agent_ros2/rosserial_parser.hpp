#pragma once
#include <cstdint>
#include <vector>
#include <optional>
#include <string>

struct RosserialFrame
{
    uint16_t topic_id;
    std::vector<uint8_t> payload;
};
// Streaming byte-wise parser for the rosserial packet format:
//
// SYNC1=0xFF, SYNC2=0xFE (Hydro+), LEN_L, LEN_H, LEN_CHK,
// TOPIC_L, TOPIC_H, PAYLOAD[LEN], MSG_CHK
//
// Checksums (Hydro+):
//  - LEN_CHK = 255 - ((LEN_L + LEN_H) % 256)
//  - MSG_CHK = 255 - ((TOPIC_L + TOPIC_H + sum(payload)) % 256)

class RosserialParser
{
public:
    RosserialParser();
    std::vector<RosserialFrame> ingest(const uint8_t *data, size_t len);

    void reset();
    std::string last_error() const  { return last_error_; }

private:
    enum class State
    {
        FIND_SYNC1,
        FIND_SYNC2,
        READ_LEN_L,
        READ_LEN_H,
        READ_LEN_CHK,
        READ_TOPIC_L,
        READ_TOPIC_H,
        READ_PAYLOAD,
        READ_MSG_CHK
    };

    // helpers
    static inline uint8_t checksum8_complement(uint32_t sum_mod256);
    void set_error(const std::string &msg);

    State state_;
    std::string last_error_;

    // working field
    uint16_t msg_len_;
    uint8_t len_l;
    uint8_t len_h;
    uint8_t expected_len_chk;

    uint8_t topic_id_;
    uint8_t topic_l_;
    uint8_t topic_h_;
    std::vector<uint8_t> payload_;
};
