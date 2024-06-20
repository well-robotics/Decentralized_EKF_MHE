#ifndef MHE_EST_HPP
#define MHE_EST_HPP

// #include <rclcpp/rclcpp.hpp>
#include <Eigen/Sparse>
#include <thread>

#include "../include/MheSrb.hpp"
#include "../include/EigenUtils.hpp"
#include "../include/spline/Bezier_simple.hpp"

using namespace Eigen;
// enum estimation_type {
//     MHE = 0,
//     KF = 1,
// };

struct robot_params
{
    // ros params
    std::vector<double> p_process_std_;
    std::vector<double> accel_input_std_;
    std::vector<double> accel_bias_std_;
    std::vector<double> gyro_input_std_;

    std::vector<double> quaternion_ib_;
    std::vector<double> p_ib_;

    int num_legs_;
    int leg_odom_type_;
    std::vector<double> joint_position_std_;
    std::vector<double> joint_velocity_std_;
    std::vector<double> foot_slide_std_;
    std::vector<double> foot_swing_std_;
    double contact_effort_theshold_;

    std::vector<double> p_init_std_;
    std::vector<double> v_init_std_;
    std::vector<double> foot_init_std_;
    std::vector<double> accel_bias_init_std_;

    std::vector<double> vo_p_std_;

    // estimator params
    int rate_;
    int N_;
    int est_type_;

    // osqp params
    double rho_;
    double alpha_;
    double delta_;
    double sigma_;
    bool verbose_;
    bool adaptRho_;
    bool polish_;
    int maxQPIter_;
    double realtiveTol_;
    double absTol_;
    double primTol_;
    double dualTol_;
    double timeLimit_;
};

struct robot_store
{
    // IMU
    double imu_time_;
    Vector3d accel_b_;
    Vector3d angular_b_;

    // Encoder & Contact
    VectorXd joint_states_position_;
    VectorXd joint_states_velocity_;
    VectorXd joint_states_effort_;
    VectorXd contact_;

    MatrixXd p_imu_2_foot_;
    MatrixXd J_imu_2_foot_;

    // VO
    double vo_time_pre_;
    double vo_time_now_;
    bool vo_new_ = false;
    Vector3d vo_p_body_pre_2_body_;
    Quaterniond vo_quaternion_;

    // Decentralized Filter
    Quaterniond quaternion_;
    Quaterniond offset_quaternion_;

    Vector3d gt_p_;
    Vector3d gt_v_s_;
};

class DecentralizedEstimation
{
public:
    DecentralizedEstimation();

    void initialize(std::shared_ptr<robot_store> sub_ptr, std::shared_ptr<robot_params> params_ptr);
    void update(int T);
    void reset();

private:
    double dt_;
    int N_;
    int est_type_;
    Vector3d gravity_;
    double contact_effort_theshold_ = 150.0; // if using theshold to detect contact
    MatrixXd R_ib_ = Matrix3d::Identity(3,3);   // Go1 body frame is choosen to be Unitree_URDF_center frame during codegen
    MatrixXd p_ib_ = MatrixXd::Zero(1, 3);

private:
    std::shared_ptr<robot_store> robot_sub_ptr_; // robot instantaneous sensory data store
    std::shared_ptr<robot_params> params_ptr_;
    MHEproblem mhe_qp_;
    Bezier vo_curve_;

private:
    //---------------------------------------------------------------
    // Current Measurements
    Vector3d accel_s_;
    Vector3d angular_b_;

    MatrixXd p_imu_2_foot_ = MatrixXd::Zero(12, 1);
    MatrixXd J_imu_2_foot_ = MatrixXd::Zero(12, 3);

    VectorXd joint_position_;
    VectorXd joint_velocity_;
    VectorXd joint_effort_;
    Vector4d contact_ = Vector4d::Zero();

    //---------------------------------------------------------------
    // Input && Measurement stack
    std::vector<int> discrete_time_stack;
    std::vector<double> imu_time_stack_;
    std::vector<Vector3d> accel_s_input_stack_;    // Spatial acceleration stacks; haven't correct for bias;
    std::vector<Vector3d> angular_b_input_stack_;  // Body gyro stacks; haven't correct for bias;
    std::vector<Matrix3d> R_input_rotation_stack_; // Spatial orientation stacks; R_sb, body frame in the world frame;

    std::vector<MatrixXd> p_imu_2_foot_stack_;
    std::vector<MatrixXd> J_imu_2_foot_stack_;
    std::vector<VectorXd> joint_velocity_stack_;
    std::vector<Vector4d> contact_input_stack_; // Contact stacks; 0, no contact, 1, contact

    //---------------------------------------------------------------
    // VO params
    std::vector<int> vo_insert_idx_stack_;
    std::vector<int> vo_insert_discrete_time_stack_;

    Vector3d vo_p_body_pre_2_body_ = Vector3d::Zero();
    bool vo_to_be_processed_flag_ = false;
    Matrix3d R_vo_sb_pre_ = Matrix3d::Zero();

    // Debug stack
    // std::vector<Vector3d> vo_pose_body_pre_2_body_stack;
    // std::vector<Matrix3d> R_vo_sb_rotation_stack_; // Spatial orientation stacks; R_sb, body frame in the world frame when got camera;
    // std::vector<double> vo_time_stack_;
    // std::vector<int> vo_sychron_time_stack_;

private:
    //---------------------------------------------------------------
    // Measurement cost&constraints term
    // 0.5 * || v_i ||^2 _{Q_meas_}
    // A_meas x_i - b_meas - v_i = 0
    int leg_odom_type_;

    int num_legs_;

    int dim_meas_;

    SparseMatrix<double> Identity_meas_;

    // A_meas:
    //   [   I  0   0;  ]
    SparseMatrix<double> A_meas_;

    // b_meas:
    // [    R_sb * leg_imu_2_foot_b(2)  ;   ]
    VectorXd b_meas_;

    // Q_meas_:
    //  [   R_sb * ( C_foot )^{-1} * R_sb';    ]
    SparseMatrix<double> Q_meas_;
    Matrix3d C_encoder_position_ = Matrix3d::Zero();
    Matrix3d C_encoder_velocity_ = Matrix3d::Zero();
    Matrix3d C_gyro_ = Matrix3d::Zero();
    Matrix3d C_foot_swing_ = Matrix3d::Zero();
    Matrix3d Q_foot_swing_ = Matrix3d::Zero();

    //---------------------------------------------------------------
    // Camera cost&constraints term
    // A_cam_pre * x_k - A_cam_now * x_k+1 - vcam_k = b_cam_k
    int dim_cam_;

    SparseMatrix<double> Identity_cam_meas_;

    // A_cam_pre:
    //   [  I  0   0   0;];
    SparseMatrix<double> A_cam_pre_;
    // A_cam_now:
    //   [  I  0   0   0;];
    SparseMatrix<double> A_cam_now_;

    // b_meas:
    // [    - R_sb_pre * vo_realtive_p  ;   ]
    VectorXd b_cam_;

    // Q_cam_pre:
    //  [   R_sb_pre * (Q_vo_p_)^{-1} * R_sb_pre'  ;  ]
    SparseMatrix<double> Q_cam_;
    Matrix3d Q_vo_p_ = Matrix3d::Zero();

private:
    //---------------------------------------------------------------
    // Dynamics cost terms
    // 0.5 * ||w_i||^2 _{Q_dyn}
    // A_dyn_ x_i - x_{i+1} - b_dyn -w_i = 0
    int dim_state_;

    SparseMatrix<double> Identity_dyn_;

    // A_dyn_:
    // [    I   dt* I   - 0.5 * dt^2 * R_sb;  ]
    // [    0   I       - dt * R_sb        ;  ]
    // [    0   0       I                  ;  ]
    SparseMatrix<double> A_dyn_;

    // b_dyn:
    // [    - 0.5 * dt^2 * accel_s; ]   p_s
    // [    - dt * accel_s        ; ]   v_s
    // [    0                     ; ]   accel_bias_b
    VectorXd b_dyn_;

    // Q_dyn:
    // [    Q_p    0    0;       ]
    // [    0      Q_v  0;       ]
    // [    0      0    Q_accel_bias; ]
    SparseMatrix<double> Q_dyn_; // Q: gains matrix

    Matrix3d C_p_ = Matrix3d::Zero();
    Matrix3d C_accel_ = Matrix3d::Zero();
    Matrix3d C_foot_slide_ = Matrix3d::Zero();
    Matrix3d Q_foot_slide_ = Matrix3d::Zero();
    Matrix3d C_accel_bias_ = Matrix3d::Zero();
    Matrix3d Q_accel_bias_ = Matrix3d::Zero(); // Gain process accel_bias

    //---------------------------------------------------------------
    // Prior cost term
    // 0.5 * || x_0 - x_prior_0 ||^2 _{Q_prior_0}

    // x_prior:
    // [    0                       ;   ]   p_s
    // [    0                       ;   ]   v_s
    // [    0                       ;   ]   accel_bias_b
    VectorXd x_prior_;

    // Q_prior_0:
    //   [  Q_p_init  0          0                ;   ]
    //   [  0         Q_v_init   0                ;   ]
    //   [  0         0          Q_accel_bias_init;   ]
    SparseMatrix<double> Q_prior_; // Gain prior

public:
    Matrix3d R_sb_;

    //---------------------------------------------------------------
    // Moving horizon estimation
    VectorXd x_MHE_;
    Vector3d v_MHE_b_ = Vector3d::Zero();
    //---------------------------------------------------------------
    // Kalmanfilter
    VectorXd x_KF_;
    MatrixXd C_KF_;
    MatrixXd K_KF_;
    Vector3d v_KF_b_ = Vector3d::Zero();

    Vector3d p_vo_accmulate_ = Vector3d::Zero();

private:
    void InitializeMHE();
    void UpdateMHE(int discrete_time);
    void GetMeasurement(int discrete_time);
    void UpdateVOConstraints(int discrete_time);

    void InitializeKF();
    void UpdateKF();
    void MarginalizeKF();

    void StdVec2CovMat(const std::vector<double> &std, Matrix3d &Cov);   // Wrapper convert Vector3d std to Diag Covariance Matrix
    void StdVec2GainMat(const std::vector<double> &std, Matrix3d &Gain); // Wrapper convert Vector3d std to Diag Gain Matrix
    void Log2txt(const MatrixXd matrix, std::string filename);
    void tic(std::string str, int mode = 0);
    void toc(std::string str);
};
#endif // MHE_EST_HPP