#ifndef MHE_EST_HPP
#define MHE_EST_HPP

// #include <rclcpp/rclcpp.hpp>
#include "../include/MheSrb.hpp"
#include "../include/EigenUtils.hpp"
#include <Eigen/Sparse>
#include <thread>


using namespace Eigen;
// enum estimation_type {
//     MHE = 0,
//     KF = 1,
// };

struct robot_params
{

    // ros params
    std::vector<double> p_process_std_;
    std::vector<double> v_process_std_;
    std::vector<double> accel_input_std_;
    std::vector<double> accel_input_contact_std_;
    std::vector<double> accel_input_impact_std_;

    std::vector<double> foothold_slide_std_;
    std::vector<double> foothold_velocity_std_;
    std::vector<double> accel_bias_std_;

    double foot_swing_std_;

    std::vector<double> lidar_std_;
    double lidar_z_std_;

    std::vector<double> p_init_std_;
    std::vector<double> v_init_std_;
    std::vector<double> vo_pose_std_;

    double foot_init_std_;
    double accel_bias_init_std_;

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
    double grf_threshold_;

    double primTol_;
    double dualTol_;
    double timeLimit_;
};

struct robot_store
{
    // vo inf
    double delta_t_;

    Vector3d accel_b_;
    Vector3d angular_b_;

    bool contact_left_;

    bool contact_right_;

    Matrix4d Kinematics_left_;
    Matrix4d Kinematics_right_;

    Matrix<double, 6, 6> Cov_left_;
    Matrix<double, 6, 6> Cov_right_;

    Quaterniond quaternion_;
    Matrix3d Rotation_;
};

class DecentralizedEstimation
{

private:
    int dim_state_ = 15;
    int dim_meas_ = 6;

public:
    DecentralizedEstimation();

    void initialize(std::shared_ptr<robot_store> sub_ptr, std::shared_ptr<robot_params> params_ptr);
    void update(int T);
    void reset();

    int est_type_;

    Matrix3d R_sb_;

    Matrix3d R_ib_ = Matrix3d::Identity(); // Body frame is choosen to align with Imu frame during codegen

    //---------------------------------------------------------------
    // Moving Horizon Estimation
    VectorXd x_MHE_ = VectorXd::Zero(15);

    //---------------------------------------------------------------
    // Kalman Filter
    VectorXd x_KF_ = VectorXd::Zero(15);

    void tic(std::string str, int mode = 0);
    void toc(std::string str);

private:
    int N_;

    std::shared_ptr<robot_store> robot_sub_ptr_; // robot instantaneous sensory data store
    std::shared_ptr<robot_params> params_ptr_;

    MHEproblem mhe_qp_;

    Vector3d gravity_;
    //---------------------------------------------------------------
    // Current measurements
    double dt_;

    Vector3d accel_s_;

    bool contact_left_ = false;
    bool contact_right_ = false;

    Vector3d p_left_ = Vector3d::Zero();  // p_imu_2_left_foot_s
    Vector3d p_right_ = Vector3d::Zero(); // p_imu_2_right_foot_s

    Matrix3d Cov_left_ = Matrix3d::Identity();
    Matrix3d Cov_right_ = Matrix3d::Identity();

    //---------------------------------------------------------------
    // Input && Measurement stack
    std::vector<double> dt_stack_;        

    std::vector<Vector3d> accel_s_input_stack_;    // Spatial acceleration stacks; haven't correct for bias;
    std::vector<Matrix3d> R_input_rotation_stack_; // Spatial orientation stacks; R_sb, body frame orientation in the spatial frame;

    std::vector<bool> contact_left_input_stack_;     // Contact stacks; 0, no contact, 1, contact
    std::vector<bool> contact_right_input_stack_;    // Contact stacks; 0, no contact, 1, contact
    std::vector<Vector3d> p_left_imu_2_foot_stack_;  // Left Leg kinematics stacks
    std::vector<Vector3d> p_right_imu_2_foot_stack_; // Right Leg kinematics stacks
    std::vector<Matrix3d> cov_left_input_stack_;
    std::vector<Matrix3d> cov_right_input_stack_;

private:

    double infinite_ = 1e6;

    //---------------------------------------------------------------
    // Dynamics cost terms
    // 0.5 * || w_i ||^2_{Q_dyn_}
    // A_dyn_ x_i - x_{i+1} - b_dyn_ -w_i = 0
    SparseMatrix<double> Identity_dyn_;

    // A_dyn_:
    // [    I   dt* I   0   0   - 0.5 * dt^2 * R_sb;  ]
    // [    0   I       0   0   - dt * R_sb        ;  ]
    // [    0   0       I   0   0                  ;  ]
    // [    0   0       0   I   0                  ;  ]
    // [    0   0       0   0   I                  ;  ]

    SparseMatrix<double> A_dyn_;

    // b_dyn_:
    // [    - 0.5 * dt^2 * accel_s; ]   p_s
    // [    - dt * accel_s        ; ]   v_s
    // [    0                     ; ]   p_foot_s
    // [    0                     ; ]   p_foot_s
    // [    0                     ; ]   accel_bias_b
    VectorXd b_dyn_ = VectorXd::Zero(dim_state_);

    // Q_dyn_:
    // If no contact; Q_foot = 0 * R_sb* Q_foot_slide_ * R_sb'
    // [    Q_p    0    0                                          0                                          0;       ]
    // [    0      Q_v  0                                          0                                          0;       ]
    // [    0      0    contact * R_sb* Q_foot_slide_ * R_sb'       0                                          0;       ]
    // [    0      0    0                                          contact * R_sb* Q_foot_slide_ * R_sb'       0;       ]
    // [    0      0    0                                          0                                          Q_accel_bias_; ]
    SparseMatrix<double> Q_dyn_; // Q: gains matrix

    Matrix3d Q_foot_slide_ = Matrix3d::Zero(); // Gain process foot, small for little slippery
    Matrix3d Q_accel_bias_ = Matrix3d::Zero(); // Gain process accel_bias

    Matrix3d C_accel_ = Matrix3d::Zero();

    Matrix3d C_p_ = Matrix3d::Zero();
    Matrix3d C_foot_slide_ = Matrix3d::Zero();
    Matrix3d C_accel_bias_ = Matrix3d::Zero();

    //---------------------------------------------------------------
    // Prior cost term
    // 0.5 * || x_0 - x_prior_0 ||^2 _{Q_prior_0}

    // x_prior_:
    // [    0                       ;   ]   p_s
    // [    0                       ;   ]   v_s
    // [    R_sb * leg_imu_2_foot_b ;   ]   p_foot_s
    // [    0                       ;   ]   accel_bias_b
    VectorXd x_prior_ = VectorXd::Zero(dim_state_);

    // Q_prior_:
    //   [  Q_p_init  0          0             0                ;   ]
    //   [  0         Q_v_init   0             0                ;   ]
    //   [  0         0          Q_foot_init   0                ;   ]
    //   [  0         0          0             Q_accel_bias_init;   ]
    SparseMatrix<double> Q_prior_; // Gain prior

    //---------------------------------------------------------------
    // Measurement cost term
    // 0.5 * || v_i ||^2 _{Q_meas_}
    // A_meas_ x_i - b_meas_ - v_i = 0
    SparseMatrix<double> Identity_meas_;

    // A_meas_:
    //   [  -I  0   I   0   0   ;
    //      -I  0   0   I   0   ;]
    SparseMatrix<double> A_meas_;

    // b_meas_:
    // [    R_sb * leg_imu_2_foot_b(2)  ;   ]
    VectorXd b_meas_ = VectorXd::Zero(dim_meas_);

    // Q_meas_:
    //  [   R_sb * ( C_lidar )^{-1} * R_sb';    ]
    SparseMatrix<double> Q_meas_; // Q_meas_ = C_meas^{-1}

private:

    void InitializeMHE();
    void UpdateMHE(int discrete_time);
    void GetMeasurement(int discrete_time);

    void StdVec2CovMat(const std::vector<double> &std, Matrix3d &Cov);   // Wrapper convert Vector3d std to Diag Covariance Matrix
    void StdVec2GainMat(const std::vector<double> &std, Matrix3d &Gain); // Wrapper convert Vector3d std to Diag Gain Matrix
    void Log2txt(const MatrixXd matrix, std::string filename);

    //---------------------------------------------------------------
    // Kalman filter
    MatrixXd C_KF_;
    MatrixXd K_KF_;

    void InitializeKF();
    void UpdateKF();
};
#endif // MHE_EST_HPP