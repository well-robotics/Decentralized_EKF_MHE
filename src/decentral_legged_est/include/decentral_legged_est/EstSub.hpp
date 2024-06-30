#ifndef ROBOT_SUB_HPP
#define ROBOT_SUB_HPP

#include <rclcpp/rclcpp.hpp>

// ros msg
#include "sensor_msgs/msg/imu.hpp"
#include "custom_msgs/msg/vo_realtive_transform.hpp"

// CPP header(non ros)
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include "decentral_legged_est/DecentralEst.hpp"
#include "decentral_legged_est/data_logger.hpp"
#include "decentral_legged_est/EigenUtils.hpp"

using namespace Eigen;

namespace robotSub
{

    class robotSub : public rclcpp::Node
    {
    public:
        robotSub(const std::string &name);
        ~robotSub();

    private:
        DecentralizedEstimation mhe;

        void paramsWrapper();

        rclcpp::TimerBase::SharedPtr timer_;
        void timerCallback();
        int discrete_time_ = 0; // discrete time of the estimation

        // Decentralized Orientation
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orien_filter_sub;
        void orien_filter_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        Vector3d filter_euler_ = Vector3d::Zero();

        // Sparsly integrated VO
        rclcpp::Subscription<custom_msgs::msg::VoRealtiveTransform>::SharedPtr vo_sub;
        void vo_callback(const custom_msgs::msg::VoRealtiveTransform::SharedPtr msg); // Sparsly integrated VO

    private:
        Data_Logger logger;
        void init_logging();

    public:
        std::shared_ptr<robot_store> robot_store_;
        std::shared_ptr<robot_params> robot_params_;

        double time_init_ = 0;
        int imu_msg_num_ = 0;

        // inital offset of mocap
        Vector3d gt_p_offset_;
        Vector3d gt_p_;
        Vector3d gt_v_b_;

        Quaterniond gt_quaternion_;
        Vector3d gt_euler_ = Vector3d::Zero(); // [roll, pitch, yaw]
    };
}
#endif // ROBOT_SUB_HPP