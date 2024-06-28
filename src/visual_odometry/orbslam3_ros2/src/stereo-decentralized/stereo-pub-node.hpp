#ifndef __STEREO_SLAM_NODE_HPP__
#define __STEREO_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "custom_msgs/msg/vo_realtive_transform.hpp"
#include <sophus/se3.hpp>
#include <Eigen/Geometry>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"


class StereoPubNode : public rclcpp::Node
{
public:
    StereoPubNode(ORB_SLAM3::System *pSLAM, const string &strSettingsFile, const string &strDoRectify);

    ~StereoPubNode();

private:
    void GrabStereo(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);

    ORB_SLAM3::System *m_SLAM;

    using ImageMsg = sensor_msgs::msg::Image;

    bool doRectify;
    cv::Mat M1l, M2l, M1r, M2r;

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;

    rclcpp::Publisher<custom_msgs::msg::VoRealtiveTransform>::SharedPtr publish_pose_stamped;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publish_pose;

    Eigen::Isometry3d T_body_to_camera_ = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond q_body_to_camera_;

    Eigen::Isometry3d T_world_to_body_init_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_world_to_camera_pre_ = Eigen::Isometry3d::Identity();

    rclcpp::Time Cv_TimeStamp;
    rclcpp::Time Cv_TimeStamp_pre;

    int init = 0;
};

#endif
