#include <ros/ros.h>
#include <sensors/UwbRange.h>
#include <data_process/UwbProcessed.h>
#include <cctype>
#include <deque>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <limits>
#include <boost/filesystem.hpp>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>
#include <ros/package.h>

class UwbProcess
{
public:
    UwbProcess()
    {
        ns_ = ros::this_node::getNamespace();
        if (!ns_.empty() && std::isdigit(ns_.back()))
            self_id_ = ns_.back() - '0';
        drone_name_ = ns_;
        if (!drone_name_.empty() && drone_name_[0] == '/')
            drone_name_ = drone_name_.substr(1);
        ROS_INFO("[%s][UWB zero_score] started self_id=%d", ns_.c_str(), self_id_);

        ros::NodeHandle pnh("~");
        pnh.param("uav_num", uav_num_, 4);
        uav_num_ = std::max(1, uav_num_);
        int queue_size = static_cast<int>(queue_size_);
        pnh.param("window_size", queue_size, 5);
        queue_size_ = std::max(1, queue_size);
        pnh.param("z_score_threshold", z_score_threshold_, 3.0);
        pnh.param("min_stddev", min_stddev_, 0.08);
        pnh.param("min_distance", min_distance_, 0.10);
        pnh.param("max_consecutive_rejects", max_consecutive_rejects_, 5);
        pnh.param("publish_rate", publish_rate_hz_, 25.0);
        pnh.param("warmup_stddev_threshold", warmup_stddev_threshold_, 0.02);
        pnh.param("csv_dir", csv_dir_, std::string(""));
        dq_.resize(uav_num_);
        reject_count_.assign(uav_num_, 0);
        warmup_checked_.assign(uav_num_, false);

        std::string uwb_topic = ns_ + "/uwb";
        uwb_sub_ = nh_.subscribe(uwb_topic, 10, &UwbProcess::sub_callback, this);

        std::string uwb_process_topic = ns_ + "/uwb_processed";
        uwb_process_pub_ = nh_.advertise<data_process::UwbProcessed>(uwb_process_topic, 10);

        initCsvLog();

        ROS_INFO("[%s][UWB zero_score] params: uav_num=%d window=%zu z=%.2f min_std=%.3f min_dist=%.2f "
                 "max_rejects=%d warmup_std=%.3f publish_rate=%.1fHz",
                 ns_.c_str(), uav_num_, queue_size_, z_score_threshold_,
                 min_stddev_, min_distance_, max_consecutive_rejects_,
                 warmup_stddev_threshold_, publish_rate_hz_);
    }

    ~UwbProcess()
    {
        if (raw_csv_.is_open()) raw_csv_.close();
        if (proc_csv_.is_open()) proc_csv_.close();
        ROS_INFO("[%s][UWB zero_score] CSV saved to %s", ns_.c_str(), csv_dir_.c_str());
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

        std::vector<bool> included(uav_num_, false);
        for (size_t i = 0; i < n; i++)
        {
            const int target_id = static_cast<int>(msg->target_ids[i]);
            const double distance = msg->distances[i];
            const double gt_distance = getGtDistance(*msg, i);
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
            if (window.size() < queue_size_)
            {
                // 队列未满时直接接受，用于初始化统计量
                window.push_back(static_cast<float>(distance));
                accepted = true;

                // 冷启动检测: 窗口首次填满时检查标准差是否过低
                if (window.size() == queue_size_ && !warmup_checked_[target_id])
                {
                    float init_stddev = cal_stddev(window);
                    if (init_stddev < warmup_stddev_threshold_)
                    {
                        ROS_WARN("[%s][UWB zero_score] target=%d cold-start detected (stddev=%.4f < %.4f), "
                                 "clearing window",
                                 ns_.c_str(), target_id, init_stddev, warmup_stddev_threshold_);
                        window.clear();
                        window.push_back(static_cast<float>(distance));
                        warmup_checked_[target_id] = false;
                    }
                    else
                    {
                        warmup_checked_[target_id] = true;
                    }
                }
            }
            else
            {
                // Zero-score 检验: 先将新值排除在队列之外计算统计量
                float mean = cal_u(window);
                float stddev = std::max(cal_stddev(window), static_cast<float>(min_stddev_));

                if (std::abs(static_cast<float>(distance) - mean) <=
                    z_score_threshold_ * stddev)
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
                    if (reject_count_[target_id] >= max_consecutive_rejects_)
                    {
                        window.clear();
                        window.push_back(static_cast<float>(distance));
                        reject_count_[target_id] = 0;
                        warmup_checked_[target_id] = false;
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
                filtered_msg.gt_distances.push_back(gt_distance);
            }
        }

        if (!filtered_msg.target_ids.empty())
        {
            latest_filtered_msg_ = filtered_msg;
            has_latest_filtered_msg_ = true;
        }

        // CSV: 原始数据
        if (raw_csv_.is_open())
        {
            for (size_t i = 0; i < n; i++)
            {
                raw_csv_ << seq_ << ","
                         << msg->header.stamp.toSec() << ","
                         << self_id_ << ","
                         << static_cast<int>(msg->target_ids[i]) << ","
                         << msg->distances[i] << ","
                         << getGtDistance(*msg, i) << "\n";
            }
            raw_csv_.flush();
        }

        // CSV: 滤波后数据
        if (proc_csv_.is_open())
        {
            for (size_t i = 0; i < filtered_msg.target_ids.size(); i++)
            {
                proc_csv_ << seq_ << ","
                          << filtered_msg.header.stamp.toSec() << ","
                          << self_id_ << ","
                          << static_cast<int>(filtered_msg.target_ids[i]) << ","
                          << filtered_msg.distances[i] << ","
                          << getGtDistance(filtered_msg, i) << "\n";
            }
            proc_csv_.flush();
        }
        ++seq_;
        /*
        ROS_INFO_THROTTLE(3.0, "[%s][UWB zero_score] raw: stamp=%.3f n=%zu%s",
                          ns_.c_str(), msg->header.stamp.toSec(), n,
                          formatRanges(msg->target_ids, msg->distances).c_str());
        ROS_INFO_THROTTLE(3.0, "[%s][UWB zero_score] filtered: stamp=%.3f n=%zu%s",
                          ns_.c_str(), filtered_msg.header.stamp.toSec(),
                          filtered_msg.target_ids.size(),
                          formatRanges(filtered_msg.target_ids, filtered_msg.distances).c_str());
                          */
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
        ros::Rate rate(publish_rate_hz_);
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
    void initCsvLog()
    {
        // csv_dir_ 已由构造函数从 ROS 参数读取; 为空时回退到 test/logs/uwb_test
        if (csv_dir_.empty())
        {
            std::string pkg_path;
            if (!ros::package::getPath("test").empty())
                pkg_path = ros::package::getPath("test");
            else
                pkg_path = "/home/scott/swarm_localization/src/test";
            csv_dir_ = pkg_path + "/logs/uwb_test";
        }

        boost::system::error_code ec;
        boost::filesystem::create_directories(csv_dir_, ec);
        if (ec)
        {
            ROS_WARN("[%s][UWB zero_score] cannot create csv_dir: %s", ns_.c_str(), csv_dir_.c_str());
            return;
        }

        std::string raw_path = csv_dir_ + "/" + drone_name_ + "_uwb_raw.csv";
        raw_csv_.open(raw_path);
        if (raw_csv_.is_open())
        {
            raw_csv_ << std::fixed << std::setprecision(6);
            raw_csv_ << "seq,timestamp,self_id,target_id,distance,gt_distance\n";
        }

        std::string proc_path = csv_dir_ + "/" + drone_name_ + "_uwb_processed.csv";
        proc_csv_.open(proc_path);
        if (proc_csv_.is_open())
        {
            proc_csv_ << std::fixed << std::setprecision(6);
            proc_csv_ << "seq,timestamp,self_id,target_id,filtered_distance,gt_distance\n";
        }

        ROS_INFO("[%s][UWB zero_score] CSV logging to %s", ns_.c_str(), csv_dir_.c_str());
    }

    bool isValidMeasurement(int target_id, double distance) const
    {
        return target_id >= 0 &&
               target_id < uav_num_ &&
               target_id != self_id_ &&
               std::isfinite(distance) &&
               distance >= min_distance_;
    }

    template <typename MsgT>
    double getGtDistance(const MsgT &msg, size_t index) const
    {
        if (index < msg.gt_distances.size() && std::isfinite(msg.gt_distances[index]))
            return msg.gt_distances[index];
        return std::numeric_limits<double>::quiet_NaN();
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
    std::string drone_name_;
    std::string csv_dir_;
    int self_id_ = 0;
    unsigned long seq_ = 0;
    std::ofstream raw_csv_;
    std::ofstream proc_csv_;

    std::vector<std::deque<float>> dq_;
    std::vector<int> reject_count_;
    std::vector<bool> warmup_checked_;
    data_process::UwbProcessed latest_filtered_msg_;
    bool has_latest_filtered_msg_ = false;

    int uav_num_ = 4;
    size_t queue_size_ = 5;
    double z_score_threshold_ = 3.0;
    double min_stddev_ = 0.08;
    double min_distance_ = 0.05;
    int max_consecutive_rejects_ = 5;
    double publish_rate_hz_ = 25.0;
    double warmup_stddev_threshold_ = 0.02;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uwb_zero_score");
    UwbProcess node;
    node.spin();
    return 0;
}
