#ifndef ANYMAL_SUB_HPP
#define ANYMAL_SUB_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <orbslam3_msgs/msg/vo_realtive_transform.hpp>
#include <optitrack_broadcast/msg/sensor_float.hpp>
#include <optitrack_broadcast/msg/mocap.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "communication/msg/foot_imu.hpp"
#include <custom_msgs/msg/sensor_float.hpp>

// tf broadcast
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

// CPP header(non ros)
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include "../include/mheEst.hpp"
#include "../include/data_logger.hpp"

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>

using namespace Eigen;

namespace pogoxSub
{

    class pogoxSub : public rclcpp::Node
    {
    private:
        // ros
        rclcpp::Subscription<orbslam3_msgs::msg::VoRealtiveTransform>::SharedPtr vo_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_filter_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vo_pose_sub;

        // ros1
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ngimu_sub;
        rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr orientation_sub;
        rclcpp::Subscription<optitrack_broadcast::msg::SensorFloat>::SharedPtr contact_sub;
        rclcpp::Subscription<optitrack_broadcast::msg::SensorFloat>::SharedPtr lidar_sub;
        rclcpp::Subscription<optitrack_broadcast::msg::Mocap>::SharedPtr mocap_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vins_sub;
        // ros2
        rclcpp::Subscription<communication::msg::FootImu>::SharedPtr footimu_sub;
        rclcpp::Subscription<custom_msgs::msg::SensorFloat>::SharedPtr socket_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vrpn_sub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vrpn_vel_sub;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr openvins_sub;

        rclcpp::TimerBase::SharedPtr timer_;

        void paramsWrapper();
        void visualize();

        MheQuadrupedalEstimation mhe;
        // mhe_params robot_params_;
        // robot_store pogox_store;
        std::shared_ptr<robot_store> pogox_store;
        std::shared_ptr<mhe_params> robot_params_;

        int discrete_time = 0;
        int est_type_; // 0: mhe, 1: KF
        int msg_num = 0;

        Vector3d pose_;
        Vector3d pose_v_;
        Matrix3d R_sb_;
        double contact_;
        Vector3d vins_pose_ = Vector3d::Zero();
        Vector3d vins_vel_ = Vector3d::Zero();

        // offset of mocap
        double pathx;
        double pathy;
        double pathz;

        bool init_mocap = false;
        bool init_vins = false;
        bool init_orientation = false;
        bool init_orientation_vins = false;
        bool init_vo_pose = false;

        Quaterniond mocap_quaternion_;
        Quaterniond vins_quaternion_;
        Quaterniond vins_offset_;
        Quaterniond vo_pose_quaternion_;
        Quaterniond vo_pose_offset_;

        Vector3d velocity_b = Vector3d::Zero();
        Vector3d vo_pose = Vector3d::Zero();
        Vector3d lord_accel_b = Vector3d::Zero();
        Vector3d ngimu_accel_b = Vector3d::Zero();

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_EST_base_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_GT_base_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_velocity_EST_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_velocity_GT_;

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_vo_pose_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_acc_bias_;

        rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr publisher_qua;

        int init_imu = 0;
        double time_0 = 0;
        double vo_matches_ = 0.0;
        double vo_norm_ = 0.0;
        double vo_norm_used_ = 0.0;
        int sockfd;
        struct sockaddr_in servaddr1;
        Vector3d vrpn_euler = Vector3d::Zero(); 
        Vector3d filter_euler = Vector3d::Zero(); 
        Vector3d vins_euler = Vector3d::Zero(); 

    public:
        pogoxSub(const std::string &name);
        ~pogoxSub();
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void orientation_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg);
        void mocap_callback(const optitrack_broadcast::msg::Mocap::SharedPtr msg);
        void contact_callback(const optitrack_broadcast::msg::SensorFloat::SharedPtr msg);
        void lidar_callback(const optitrack_broadcast::msg::SensorFloat::SharedPtr msg);
        void imu_filter_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        void vo_callback(const orbslam3_msgs::msg::VoRealtiveTransform::SharedPtr msg);
        void vins_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void vo_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        void foot_imu_callback(const communication::msg::FootImu::SharedPtr msg);
        void vrpn_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void socket_callback(const custom_msgs::msg::SensorFloat::SharedPtr msg);
        void vrpn_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        void openvins_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void quaternionToEuler(Quaterniond quaternion, double &yaw, double &pitch, double &roll);

        void timerCallback();

        Data_Logger logger;
        void init_logging();
    };
}
#endif // ANYMAL_SUB_HPP