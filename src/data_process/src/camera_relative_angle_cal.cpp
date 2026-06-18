#include <ros/ros.h>
#include <sensors/ImageDetection.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <data_process/CameraAngle.h>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>

class CameraRelativeAngleCal
{
public:
    CameraRelativeAngleCal()
    {
        ros::NodeHandle pnh("~");
        std::string ns = ros::this_node::getNamespace();

        // 构建 camera_link 全局 frame 名 (eg. /iris_0 -> iris_0/camera_link)
        std::string ns_clean = ns;
        if (!ns_clean.empty() && ns_clean[0] == '/')
            ns_clean = ns_clean.substr(1);
        camera_frame_ = ns_clean + "/camera_link";
        pnh.param<std::string>("world_frame", world_frame_, "map");
        pnh.param<double>("tf_timeout", tf_timeout_, 0.03);

        ROS_INFO("[%s] world_frame=%s, camera_frame=%s, tf_timeout=%.3f",
                 ns.c_str(), world_frame_.c_str(), camera_frame_.c_str(), tf_timeout_);

        // 订阅 image_detection (归一化检测框)
        det_sub_ = nh_.subscribe("image_detection", 10,
                                 &CameraRelativeAngleCal::detCallback, this);

        // 订阅 camera_info (获取相机内参 K)
        cam_info_sub_ = nh_.subscribe("camera/camera_info", 10,
                                       &CameraRelativeAngleCal::camInfoCallback, this);

        // 发布 polar angles (批量, 每帧一消息)
        angle_pub_ = nh_.advertise<data_process::CameraAngle>(ns + "/camera_angle", 10);
    }

    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg)
    {
        fx_ = msg->K[0];
        fy_ = msg->K[4];
        cx_ = msg->K[2];
        cy_ = msg->K[5];
        has_cam_info_ = true;
    }

    void detCallback(const sensors::ImageDetection::ConstPtr &msg)
    {
        const size_t n_det = std::min<size_t>(
            msg->count,
            std::min({msg->x.size(), msg->y.size(), msg->confidence.size()}));

        if (n_det == 0)
        {
            publishInvalidAngles(msg, 0);
            return;
        }

        if (n_det != msg->count)
        {
            ROS_WARN_THROTTLE(10.0,
                              "ImageDetection count mismatch: count=%u x=%zu y=%zu confidence=%zu, use %zu",
                              msg->count, msg->x.size(), msg->y.size(),
                              msg->confidence.size(), n_det);
        }

        if (!has_cam_info_ || fx_ <= 1e-6f || fy_ <= 1e-6f)
        {
            ROS_WARN_THROTTLE(10.0,
                              "[camera_relative_angle_cal] no valid camera_info yet; skip frame");
            return;
        }

        // 查询 camera_link -> map 变换, 将相机射线转到 ENU 世界系.
        tf::StampedTransform transform;
        try
        {
            if (tf_timeout_ > 0.0)
            {
                tf_listener_.waitForTransform(world_frame_, camera_frame_,
                                              msg->header.stamp,
                                              ros::Duration(tf_timeout_));
            }
            tf_listener_.lookupTransform(world_frame_, camera_frame_,
                                         msg->header.stamp, transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN_THROTTLE(10.0, "[camera_relative_angle_cal] TF lookup %s -> %s failed: %s; skip frame",
                              camera_frame_.c_str(), world_frame_.c_str(), ex.what());
            return;
        }

        tf::Matrix3x3 R_world_cam = transform.getBasis();

        data_process::CameraAngle msg_out;
        msg_out.header.stamp = msg->header.stamp;
        msg_out.header.frame_id = world_frame_;

        for (size_t i = 0; i < n_det; i++)
        {
            double alpha = invalidAngle();
            double theta = invalidAngle();

            // Step 1: 归一化 [0,1] -> 像素坐标
            float u = msg->x[i] * IMG_WIDTH_;
            float v = msg->y[i] * IMG_HEIGHT_;

            // Step 2: 像素坐标 -> 相机光学帧射线方向 (归一化像平面)
            // 光学帧: X=right, Y=down, Z=forward
            tf::Vector3 ray_cam((u - cx_) / fx_, (v - cy_) / fy_, 1.0);

            // Step 3: 光学帧 (Z=forward) -> camera_link FLU (X=forward)
            // FLU_X = optical_Z, FLU_Y = -optical_X, FLU_Z = -optical_Y
            tf::Vector3 ray_link(ray_cam.z(), -ray_cam.x(), -ray_cam.y());

            // Step 4: camera_link FLU -> ENU 世界系
            tf::Vector3 ray_world = R_world_cam * ray_link;

            double x_paper = ray_world.x();  // East
            double y_paper = ray_world.y();  // North
            double z_paper = ray_world.z();  // Up
            double mag = std::sqrt(x_paper * x_paper +
                                   y_paper * y_paper +
                                   z_paper * z_paper);
            if (mag >= 1e-6 && std::isfinite(mag))
            {
                // Step 5: 极坐标角度
                alpha = wrapAngle(std::atan2(y_paper, x_paper));
                theta = std::asin(clamp(z_paper / mag, -1.0, 1.0));
            }

            msg_out.alpha.push_back(static_cast<float>(alpha));
            msg_out.theta.push_back(static_cast<float>(theta));
        }
        msg_out.count = static_cast<uint8_t>(msg_out.alpha.size());
        angle_pub_.publish(msg_out);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber det_sub_;
    ros::Subscriber cam_info_sub_;
    ros::Publisher angle_pub_;
    tf::TransformListener tf_listener_;

    std::string camera_frame_;   // eg. "iris_0/camera_link"
    std::string world_frame_;    // eg. "map" (ENU)
    double tf_timeout_ = 0.03;

    float fx_ = 0.0f, fy_ = 0.0f, cx_ = 0.0f, cy_ = 0.0f;
    bool has_cam_info_ = false;

    static constexpr float IMG_WIDTH_  = 320.0f;
    static constexpr float IMG_HEIGHT_ = 240.0f;

    void publishInvalidAngles(const sensors::ImageDetection::ConstPtr &msg, size_t n_det)
    {
        data_process::CameraAngle msg_out;
        msg_out.header.stamp = msg->header.stamp;
        msg_out.header.frame_id = world_frame_;
        msg_out.count = static_cast<uint8_t>(n_det);
        msg_out.alpha.resize(n_det, static_cast<float>(invalidAngle()));
        msg_out.theta.resize(n_det, static_cast<float>(invalidAngle()));
        angle_pub_.publish(msg_out);
    }

    static double invalidAngle()
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    static double clamp(double value, double lo, double hi)
    {
        return std::max(lo, std::min(hi, value));
    }

    static double wrapAngle(double angle)
    {
        while (angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }
        return angle;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_relative_angle_cal");
    CameraRelativeAngleCal node;
    ros::spin();
    return 0;
}
