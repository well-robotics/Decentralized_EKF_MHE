#ifndef ANYMAL_SUB_HPP
#define ANYMAL_SUB_HPP

#include <rclcpp/rclcpp.hpp>

// CPP header(non ros)
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include "../include/DecentralEst.hpp"
#include "../include/data_logger.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <boost/algorithm/string.hpp>
#include <vector>

using namespace Eigen;

namespace EstSub
{

    class EstSub : public rclcpp::Node
    {
    private:
        rclcpp::TimerBase::SharedPtr timer_;

        void paramsWrapper();

        DecentralizedEstimation estimator_;

        std::shared_ptr<robot_store> robot_store_;
        std::shared_ptr<robot_params> robot_params_;
        std::ifstream infile_meas_;
        std::ifstream infile_orien_gt_;
        std::ifstream infile_vel_gt_;
        std::ifstream infile_vel_iekf_;
        std::ifstream infile_orien_iekf_;

        int discrete_time_ = 0;

        int est_type_;

        VectorXd quaternion_ = VectorXd::Zero(4);

        Vector3d vel_gt_ = Vector3d::Zero();
        Eigen::Matrix3d Rot_gt_ = Matrix3d::Identity();
        VectorXd quaternion_gt_ = VectorXd::Zero(4);
        Vector3d vel_iekf_ = Vector3d::Zero();
        Eigen::Matrix3d Rot_iekf_ = Matrix3d::Identity();
        VectorXd quaternion_iekf_ = VectorXd::Zero(4);

        double t_ = 0;
        double t_prev_ = 0;
        Vector3d iekf_euler_ = Vector3d::Zero();
        Vector3d filter_euler_ = Vector3d::Zero();
        Vector3d gt_euler_ = Vector3d::Zero();

    public:
        EstSub(const std::string &name);
        ~EstSub();

        void timerCallback();

        Data_Logger logger_;
        void init_logging();

    public:
        double stod98(const std::string &s);
        int stoi98(const std::string &s);
        void quaternionToEuler(VectorXd &quaternion, double &yaw, double &pitch, double &roll);

    public:
        Matrix3d C_accel_ = Matrix3d::Zero();
        Matrix3d C_gyro_ = Matrix3d::Zero();
        VectorXd quternion_prior_ = VectorXd::Zero(4);
        VectorXd quaternion_pred_ = VectorXd::Zero(4);
        VectorXd quaternion_correct_ = VectorXd::Zero(4);
        MatrixXd Cov_q_ = MatrixXd::Zero(4, 4);
        MatrixXd Cov_q_pred_ = MatrixXd::Zero(4, 4);
        MatrixXd Cov_q_correct_ = MatrixXd::Zero(4, 4);
        Vector3d gravity_ = Vector3d::Zero();

    public:
        void gyro_nonlinear_predict(VectorXd &quaternion_pred, VectorXd &quaternion, Vector3d &gyro_reading, MatrixXd &Cov_q, MatrixXd &Cov_q_pred, double dt_);
        void gyro_nonlinear_correct(VectorXd &quaternion_correct, VectorXd &quaternion_pred, Vector3d &accel_readings, MatrixXd &Cov_q_pred, MatrixXd &Cov_q_correct);
        void gyro_2_Ohm(Vector3d &gyro_reading, MatrixXd &Ohm);
        void quat_2_W(VectorXd &quaternion, MatrixXd &W, double &dt);
        void quat_2_Rot(VectorXd &quaternion, MatrixXd &Rotation);
        void quat_2_H(VectorXd &quaternion, MatrixXd &H);
        void quat_norm(VectorXd &quaternion);
        void InitOrienEKF();
    };
}
#endif // ANYMAL_SUB_HPP