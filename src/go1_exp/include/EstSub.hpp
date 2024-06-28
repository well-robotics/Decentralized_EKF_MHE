#ifndef ROBOT_SUB_HPP
#define ROBOT_SUB_HPP

#include <rclcpp/rclcpp.hpp>

// ros msg
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <custom_msgs/msg/vo_realtive_transform.hpp>

#include <optitrack_broadcast/msg/mocap.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

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
    public:
        robotSub(const std::string &name);
        ~robotSub();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        void timerCallback();

        // Decentralized Orientation
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orien_filter_sub;
        void orien_filter_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        Vector3d filter_euler_ = Vector3d::Zero();

        void paramsWrapper();

    private:
        Data_Logger logger;
        void init_logging();

    private:
        DecentralizedEstimation mhe;
        std::shared_ptr<robot_store> robot_store_;
        std::shared_ptr<robot_params> robot_params_;

        int est_type_;          // 0: mhe, 1: KF

        double time_init_ = 0;
        int discrete_time_ = 0; // discrete time of the estimation
        int imu_msg_num_ = 0;

        // inital offset of mocap
        Vector3d gt_p_offset_;
        Vector3d gt_p_;
        Vector3d gt_v_b_;

        bool init_mocap_ = false;
        bool init_vo_ = false;

        Quaterniond mocap_quaternion_;
        Vector3d gt_euler_ = Vector3d::Zero(); // [roll, pitch, yaw]
    };
}
#endif // ROBOT_SUB_HPP