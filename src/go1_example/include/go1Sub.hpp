#ifndef GO1_SUB_HPP
#define GO1_SUB_HPP

#include <rclcpp/rclcpp.hpp>

// CPP header(non ros)
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include "decentral_legged_est/EigenUtils.hpp"
#include "decentral_legged_est/EstSub.hpp"

// ros msg 
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <optitrack_broadcast/msg/mocap.hpp>

// kinematics lib
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

    class go1Sub : public robotSub
    {
    private:
        // IMU 
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        
        //Leg Odometry, encoder information & kinematics & contact 
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr lo_sub;
        void lo_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

        // Ground Truth
        rclcpp::Subscription<optitrack_broadcast::msg::Mocap>::SharedPtr mocap_sub;
        void mocap_callback(const optitrack_broadcast::msg::Mocap::SharedPtr msg);

    public:
        go1Sub(const std::string &name);
        ~go1Sub();
    };
}
#endif // GO1_SUB_HPP