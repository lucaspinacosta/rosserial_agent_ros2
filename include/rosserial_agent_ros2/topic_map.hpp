#pragma once
#include <cstdint>
#include <string>
#include <unordered_map>
#include <optional>

struct TopicInfoRecord
{
    uint16_t endpoint_id;     // TopicInfo.topic_id (endpoint id)
    std::string topic_name;   // e.g. "/chatter"
    std::string message_type; // e.g. "std_msgs/String"
    std::string md5sum;       // ROS 1 message MD5
    int32_t buffer_size;      // MCU-side buffer hint
};

class TopicMap
{
public:
    void upsert(const TopicInfoRecord &r)
    {
        map_[r.endpoint_id] = r;
        name_to_id_[r.topic_name] = r.endpoint_id;
    }

    std::optional<TopicInfoRecord> by_id(uint16_t id) const
    {
        auto it = map_.find(id);
        if (it == map_.end())
            return std::nullopt;
        return it->second;
    }

private:
    std::unordered_map<uint16_t, TopicInfoRecord> map_;
    std::unordered_map<std::string, uint16_t> name_to_id_;
};
