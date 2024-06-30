#ifndef IMU_EKF_HPP
#define IMU_EKF_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// tf broadcast
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
// #include "communication/msg/foot_imu.hpp"
#include "std_msgs/msg/header.hpp"
#include <optitrack_broadcast/msg/mocap.hpp>

using namespace Eigen;

namespace orien_ekf
{

    class orien_ekf : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vo_pose_sub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_filter_;
        // rclcpp::Subscription<communication::msg::FootImu>::SharedPtr imu_ros2_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vrpn_sub;

        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_filter_euler_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_mocap_euler_;
        rclcpp::Subscription<optitrack_broadcast::msg::Mocap>::SharedPtr mocap_sub;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_vo_euler_;

        rclcpp::TimerBase::SharedPtr timer_;

        Vector3d accel_b_ = Vector3d::Zero();
        Vector3d angular_vel_b_ = Vector3d::Zero();

        int msg_num = 0;

        double dt_ = 0.002;

        bool init_vo_pose = false;
        bool init_filter = false;

        VectorXd vo_pose_quaternion_ = VectorXd::Zero(4);

        Vector3d gravity_ = Vector3d::Zero();
        VectorXd quternion_prior_;

        double time_0_;
        double imu_time_;
        double vo_time_;
        int discrete_time = 0;
        std::vector<Vector3d> angular_vel_stack_;
        std::vector<Vector3d> accel_stack_;
        std::vector<double> imu_time_stack_;
        std::vector<int> discrete_time_stack_;

        std::vector<Vector4d> filter_quaternion_stack_;
        std::vector<Matrix4d> Cov_stack_;

        bool vo_new_ = false;
        std_msgs::msg::Header imu_header;

    private:
        VectorXd quaternion_ = VectorXd::Zero(4, 0);
        VectorXd mocap_quaternion_ = VectorXd::Zero(4, 0);

        VectorXd quaternion_pred_ = VectorXd::Zero(4, 0);
        VectorXd quaternion_correct_ = VectorXd::Zero(4, 0);
        MatrixXd Cov_q_ = MatrixXd::Zero(4, 4);
        MatrixXd Cov_q_pred_ = MatrixXd::Zero(4, 4);
        MatrixXd Cov_q_correct_ = MatrixXd::Zero(4, 4);

        Matrix3d C_accel_ = Matrix3d::Zero();
        Matrix3d C_gyro_ = Matrix3d::Zero();
        Matrix4d C_vo_ = MatrixXd::Zero(4, 4);

        int init_ = 0;
        int init_mocap = 0;
        int init_vo = 0;
        int init_imu = 0;
        VectorXd vo_mocap_quaternion_offset_ = VectorXd::Zero(4, 0);

    public:
        orien_ekf(const std::string &name);

        void vo_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        // void imu_ros2_callback(const communication::msg::FootImu::SharedPtr msg);

        void gyro_nonlinear_predict(VectorXd &quaternion_pred, VectorXd &quaternion, Vector3d &gyro_reading, MatrixXd &Cov_q, MatrixXd &Cov_q_pred);
        void gyro_nonlinear_correct(VectorXd &quaternion_correct, VectorXd &quaternion_pred, Vector3d &accel_readings, MatrixXd &Cov_q_pred, MatrixXd &Cov_q_correct);
        void vo_nonlinear_correct(VectorXd &quaternion_correct, VectorXd &quaternion_pred, VectorXd &quaternion_vo, MatrixXd &Cov_q_pred, MatrixXd &Cov_q_correct);

        void gyro_2_Ohm(Vector3d &gyro_reading, MatrixXd &Ohm);
        void quat_2_W(VectorXd &quaternion, MatrixXd &W);
        void quat_2_Rot(VectorXd &quaternion, MatrixXd &Rotation);
        void quat_2_H(VectorXd &quaternion, MatrixXd &H);
        void quat_norm(VectorXd &quaternion);
        void get_measurement();
        void timerCallback();
        void quaternionToEuler(VectorXd quaternion, double &yaw, double &pitch, double &roll);
        void quat_mul(VectorXd &quaternion_left, VectorXd &quaternion_right, VectorXd &quaternion_output);
        void quat_inv(VectorXd &quaternion, VectorXd &quaternion_out);
        void vrpn_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void mocap_callback(const optitrack_broadcast::msg::Mocap::SharedPtr msg);
    };
}
#endif // NGIMU_TF_HPP