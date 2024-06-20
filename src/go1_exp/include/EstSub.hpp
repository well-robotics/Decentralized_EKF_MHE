#ifndef ROBOT_SUB_HPP
#define ROBOT_SUB_HPP

#include <rclcpp/rclcpp.hpp>

// Ros msg header
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <orbslam3_msgs/msg/vo_realtive_transform.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <optitrack_broadcast/msg/mocap.hpp>

// CPP header(non ros)
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include "../include/DecentralEst.hpp"
#include "../include/data_logger.hpp"
#include "../include/EigenUtils.hpp"

#include "../include/Expressions/FL_foot.hh"
#include "../include/Expressions/FR_foot.hh"
#include "../include/Expressions/RL_foot.hh"
#include "../include/Expressions/RR_foot.hh"
#include "../include/Expressions/J_FL.hh"
#include "../include/Expressions/J_FR.hh"
#include "../include/Expressions/J_RL.hh"
#include "../include/Expressions/J_RR.hh"

using namespace Eigen;

namespace robotSub
{

    class robotSub : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr lo_sub;

        // Sparsly integrated VO
        rclcpp::Subscription<orbslam3_msgs::msg::VoRealtiveTransform>::SharedPtr vo_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vo_p_sub;

        // Decentralized Orientation
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orien_filter_sub;

        // Ground Truth
        // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_sub;
        rclcpp::Subscription<optitrack_broadcast::msg::Mocap>::SharedPtr mocap_sub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mocap_vel_sub;

        rclcpp::TimerBase::SharedPtr timer_;

    private:
        void paramsWrapper();

        DecentralizedEstimation mhe;

        std::shared_ptr<robot_store> robot_store_;
        std::shared_ptr<robot_params> robot_params_;

        double time_0_ = 0;
        int discrete_time = 0; // discrete time of the estimation
        int est_type_;         // 0: mhe, 1: KF
        int msg_num = 0;

        // inital offset of mocap
        Vector3d gt_p_offset_;
        Vector3d gt_p_;
        Vector3d gt_v_b_;

        bool init_mocap_ = false;
        bool init_vo_ = false;

        Quaterniond mocap_quaternion_;

        Vector3d vo_p_ = Vector3d::Zero();
        Quaterniond vo_quaternion_;
        Quaterniond vo_quaternion_offset_;

        Vector3d gt_euler_ = Vector3d::Zero(); // [roll, pitch, yaw]
        Vector3d filter_euler_ = Vector3d::Zero();

    public:
        robotSub(const std::string &name);
        ~robotSub();
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void lo_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

        // Sparsly integrated VO
        void vo_p_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void vo_callback(const orbslam3_msgs::msg::VoRealtiveTransform::SharedPtr msg);

        // Decentralized Orientation
        void orien_filter_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        // Ground Truth
        void mocap_callback(const optitrack_broadcast::msg::Mocap::SharedPtr msg);

        // void mocap_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        // void mocap_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        void timerCallback();

    public:
        Data_Logger logger;
        void init_logging();
    };
}
#endif // ROBOT_SUB_HPP