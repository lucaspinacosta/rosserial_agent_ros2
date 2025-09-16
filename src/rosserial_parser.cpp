#include "rosserial_agent_ros2/rosserial_parser.hpp"

RosserialParser::RosserialParser() { reset(); }

void RosserialParser::reset()
{
    state_ = State::FIND_SYNC1;
    last_error_.clear();
    msg_len_ = 0;
    len_l = len_h = expected_len_chk = 0;
    topic_id_ = 0;
    topic_l_ = topic_h_ = 0;
    payload_.clear();
}

inline uint8_t RosserialParser::checksum8_complement(uint32_t sum_mod256)
{
    // 8-bit two's complement (aka 255 - (sum % 256))
    return static_cast<uint8_t>(0xFFu - (sum_mod256 & 0xFFu));
}

void RosserialParser::set_error(const std::string &msg)
{
    last_error_ = msg;
}

std::vector<RosserialFrame> RosserialParser::ingest(const uint8_t *data, size_t len)
{
    std::vector<RosserialFrame> out;
    for (size_t i = 0; i < len; i++)
    {
        uint8_t b = data[i];
        switch (state_)
        {
        case State::FIND_SYNC1:
            if (b == 0xFF)
                state_ = State::FIND_SYNC2;
            break;

        case State::FIND_SYNC2:
            if (b == 0xFE || b == 0xFF)
            {
                // Alllow 0xFE (Hydro+) and 0xFF (Groovy-era)
                state_ = State::FIND_SYNC1;
            }
            else
            {
                // restart
                state_ = State::FIND_SYNC1;
            }
            break;
        case State::READ_LEN_L:
            len_l = b;
            state_ = State::READ_LEN_H;
            break;
        case State::READ_LEN_H:
            len_h = b;
            msg_len_ = static_cast<uint16_t>(len_l) | (static_cast<uint16_t>(len_h) << 8);
            state_ = State::READ_LEN_CHK;
            break;
        case State::READ_LEN_CHK:
        {
            uint8_t want = checksum8_complement(static_cast<uint32_t>(len_l) + static_cast<uint32_t>(len_h));
            if (b != want)
            {
                set_error("Length checksum mismatch");
                state_ = State::FIND_SYNC1;
                break;
            }
            state_ = State::READ_TOPIC_L;
            break;
        }
        case State::READ_TOPIC_L:
            topic_l_ = b;
            state_ = State::READ_TOPIC_H;
            break;
        case State::READ_TOPIC_H:
            topic_h_ = b;
            topic_id_ = static_cast<uint16_t>(topic_l_) | (static_cast<uint16_t>(topic_h_) << 8);
            payload_.clear();
            payload_.reserve(msg_len_);
            if (msg_len_ == 0)
            {
                state_ = State::READ_MSG_CHK; // Empy Service ( zero-length payload)
            }
            else
            {
                state_ = State::READ_PAYLOAD;
            }
            break;

        case State::READ_PAYLOAD:
            payload_.push_back(b);
            if (payload_.size() >= msg_len_)
            {
                state_ = State::READ_MSG_CHK;
            }
            break;
            case State::READ_MSG_CHK:
            {
                // Compute checksum [topic bytes + payload]
                uint32_t sum = static_cast<uint32_t>(topic_l_) + static_cast<uint32_t>(topic_h_);
                for (auto v : payload_)
                    sum += static_cast<uint32_t>(v);
                uint8_t want = checksum8_complement(sum);
                if (b != want)
                {
                    set_error("Message checksum mismatch");
                    
                    // drop and resync
                    state_ = State::FIND_SYNC1;
                    break;
                }
                
                // Got a full message!
                RosserialFrame f;
                f.topic_id = topic_id_;
                f.payload = std::move(payload_);
                out.emplace_back(std::move(f));

                // reset dor next message
                payload_.clear();
                state_ = State::FIND_SYNC1;
                break;
            }
        }
    }
    return out;
}
