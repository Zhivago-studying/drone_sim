#include <ros/ros.h>
#include <sensors/UwbRange.h>
#include <data_process/UwbProcessed.h>
#include <cctype>
#include <deque>
#include <cmath>
#include <sstream>
#include <vector>

class UwbProcess
{
public:
    UwbProcess()
    {
        ns_ = ros::this_node::getNamespace();
        if (!ns_.empty() && std::isdigit(ns_.back()))
            self_id_ = ns_.back() - '0';
        ROS_INFO("[%s][UWB zero_score] started self_id=%d", ns_.c_str(), self_id_);

        std::string uwb_topic = ns_ + "/uwb";
        uwb_sub_ = nh_.subscribe(uwb_topic, 10, &UwbProcess::sub_callback, this);

        std::string uwb_process_topic = ns_ + "/uwb_processed";
        uwb_process_pub_ = nh_.advertise<data_process::UwbProcessed>(uwb_process_topic, 10);
    }

    void sub_callback(const sensors::UwbRange::ConstPtr &msg)
    {
        const size_t n = std::min(msg->distances.size(), msg->target_ids.size());
        data_process::UwbProcessed filtered_msg;
        filtered_msg.header = msg->header;
        if (filtered_msg.header.stamp.isZero())
            filtered_msg.header.stamp = ros::Time::now();
        filtered_msg.header.frame_id = msg->header.frame_id.empty() ? "uwb_link" : msg->header.frame_id;

        if (msg->distances.size() != msg->target_ids.size())
        {
            ROS_WARN_THROTTLE(10.0,
                              "[%s][UWB zero_score] input size mismatch: target_ids=%zu distances=%zu, use %zu",
                              ns_.c_str(), msg->target_ids.size(), msg->distances.size(), n);
        }

        bool included[4] = {false, false, false, false};
        for (size_t i = 0; i < n; i++)
        {
            const int target_id = static_cast<int>(msg->target_ids[i]);
            const double distance = msg->distances[i];
            if (!isValidMeasurement(target_id, distance))
            {
                ROS_WARN_THROTTLE(5.0,
                                  "[%s][UWB zero_score] reject invalid target=%d distance=%.4f",
                                  ns_.c_str(), target_id, distance);
                continue;
            }
            if (included[target_id])
            {
                ROS_WARN_THROTTLE(5.0,
                                  "[%s][UWB zero_score] reject duplicate target=%d",
                                  ns_.c_str(), target_id);
                continue;
            }

            auto &window = dq_[target_id];
            bool accepted = false;
            if (window.size() < QUEUE_SIZE_)
            {
                // 队列未满时直接接受，用于初始化统计量
                window.push_back(static_cast<float>(distance));
                accepted = true;
            }
            else
            {
                // Zero-score 检验: 先将新值排除在队列之外计算统计量
                float mean = cal_u(window);
                float stddev = std::max(cal_stddev(window), MIN_STDDEV_);

                if (std::abs(static_cast<float>(distance) - mean) <
                    Z_SCORE_THRESHOLD_ * stddev)
                {
                    window.pop_front();
                    window.push_back(static_cast<float>(distance));
                    accepted = true;
                }
                else
                {
                    ++reject_count_[target_id];
                    ROS_WARN_THROTTLE(3.0,
                                      "[%s][UWB zero_score] reject target=%d dist=%.4f mean=%.4f std=%.4f reject_count=%d",
                                      ns_.c_str(), target_id, distance, mean, stddev,
                                      reject_count_[target_id]);
                    if (reject_count_[target_id] >= MAX_CONSECUTIVE_REJECTS_)
                    {
                        window.clear();
                        window.push_back(static_cast<float>(distance));
                        reject_count_[target_id] = 0;
                        accepted = true;
                        ROS_WARN("[%s][UWB zero_score] reset target=%d filter window after consecutive rejects",
                                 ns_.c_str(), target_id);
                    }
                }
            }

            if (accepted)
            {
                reject_count_[target_id] = 0;
                included[target_id] = true;
                filtered_msg.target_ids.push_back(static_cast<uint8_t>(target_id));
                filtered_msg.distances.push_back(cal_u(window));
            }
        }

        if (!filtered_msg.target_ids.empty())
        {
            latest_filtered_msg_ = filtered_msg;
            has_latest_filtered_msg_ = true;
        }

        ROS_INFO_THROTTLE(3.0, "[%s][UWB zero_score] raw: stamp=%.3f n=%zu%s",
                          ns_.c_str(), msg->header.stamp.toSec(), n,
                          formatRanges(msg->target_ids, msg->distances).c_str());
        ROS_INFO_THROTTLE(3.0, "[%s][UWB zero_score] filtered: stamp=%.3f n=%zu%s",
                          ns_.c_str(), filtered_msg.header.stamp.toSec(),
                          filtered_msg.target_ids.size(),
                          formatRanges(filtered_msg.target_ids, filtered_msg.distances).c_str());
    }

    float cal_u(const std::deque<float> &dq)
    {
        if (dq.empty()) return 0.0f;
        float sum = 0.0f;
        for (const auto &val : dq)
            sum += val;
        return sum / dq.size();
    }

    float cal_stddev(const std::deque<float> &dq)
    {
        if (dq.empty()) return 0.0f;
        float u = cal_u(dq);
        float temp = 0.0f;
        for (const auto &val : dq)
            temp += (val - u) * (val - u);
        return std::sqrt(temp / dq.size());
    }

    void spin()
    {
        ros::Rate rate(25.0);
        while (ros::ok())
        {
            ros::spinOnce();

            if (!has_latest_filtered_msg_)
            {
                rate.sleep();
                continue;
            }

            uwb_process_pub_.publish(latest_filtered_msg_);

            ROS_INFO_THROTTLE(3.0, "[%s][UWB zero_score] pub: stamp=%.3f n=%zu%s",
                              ns_.c_str(), latest_filtered_msg_.header.stamp.toSec(),
                              latest_filtered_msg_.target_ids.size(),
                              formatRanges(latest_filtered_msg_.target_ids,
                                           latest_filtered_msg_.distances).c_str());

            rate.sleep();
        }
    }

private:
    bool isValidMeasurement(int target_id, double distance) const
    {
        return target_id >= 0 &&
               target_id < UAV_NUM_ &&
               target_id != self_id_ &&
               std::isfinite(distance) &&
               distance >= MIN_DISTANCE_;
    }

    std::string formatRanges(const std::vector<uint8_t> &target_ids,
                             const std::vector<double> &distances) const
    {
        std::ostringstream oss;
        const size_t n = std::min(target_ids.size(), distances.size());
        for (size_t i = 0; i < n; ++i)
        {
            oss << " | target=" << static_cast<int>(target_ids[i])
                << " dist=" << distances[i];
        }
        return oss.str();
    }

    ros::NodeHandle nh_;
    ros::Subscriber uwb_sub_;
    ros::Publisher uwb_process_pub_;
    std::string ns_;
    int self_id_ = 0;

    std::deque<float> dq_[4];
    int reject_count_[4] = {0, 0, 0, 0};
    data_process::UwbProcessed latest_filtered_msg_;
    bool has_latest_filtered_msg_ = false;

    static constexpr int UAV_NUM_ = 4;
    static constexpr size_t QUEUE_SIZE_ = 5;
    static constexpr float Z_SCORE_THRESHOLD_ = 3.0f;
    static constexpr float MIN_STDDEV_ = 0.08f;
    static constexpr double MIN_DISTANCE_ = 0.05;
    static constexpr int MAX_CONSECUTIVE_REJECTS_ = 5;
};

constexpr float UwbProcess::MIN_STDDEV_;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uwb_zero_score");
    UwbProcess node;
    node.spin();
    return 0;
}
