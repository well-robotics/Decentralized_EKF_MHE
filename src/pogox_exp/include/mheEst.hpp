#ifndef MHE_EST_HPP
#define MHE_EST_HPP

// #include <rclcpp/rclcpp.hpp>
#include <Eigen/Sparse>
#include <thread>

#include "../include/MheSrb.hpp"
#include "../include/EigenUtils.hpp"
#include "../include/spline/Bezier_simple.hpp"

#include "../include/frost_codegen/pToe.hh"
#include "../include/frost_codegen/J_Toe.hh"

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
    double imu_time_;
    double vo_time_pre_;
    double vo_time_now_;
    bool vo_new_ = false;
    Vector3d vo_pose_body_pre_2_body_;
    Quaterniond vo_quaternion_;
    double vo_matches_;

    // general inf
    Vector3d accel_b_;

    double distance_;

    double stance_;

    Quaterniond quaternion_;
    Quaterniond offset_quaternion_;

    Vector3d pose_;
    Vector3d v_sb_;
};

class DecentralizedEstimation
{

public:
    DecentralizedEstimation();

    void initialize(std::shared_ptr<robot_store> sub_ptr, std::shared_ptr<robot_params> params_ptr);
    void update(int T);
    void reset();

private:
    double dt;
    int N;
    int est_type_;
    Vector3d gravity_;
    Matrix3d R_ib_ = Matrix3d::Identity(); // Pogox body frame is choosen to be imu frame during codegen
    MatrixXd p_ib_ = MatrixXd::Zero(1, 3);

private:
    std::shared_ptr<robot_store> robot_sub_ptr_; // robot instantaneous sensory data store
    std::shared_ptr<robot_params> params_ptr_;
    MHEproblem mhe_qp;
    Bezier vo_curve;

private:
    //---------------------------------------------------------------
    // Current measurements
    Vector3d accel_s_;
    Vector3d p_foot_; // p_imu_2_foot_s
    double Contact_;
    double Contact_sum_ = 0;

    //---------------------------------------------------------------
    // Input && Measurement stack
    std::vector<int> discrete_time_stack; // Contact stacks; 0, no contact, 1, contact
    std::vector<double> imu_time_stack;
    std::vector<Vector3d> accel_s_input_stack; // Spatial acceleration stacks; haven't correct for bias;
    std::vector<Matrix3d> R_input_rotation_stack; // Spatial orientation stacks; R_sb, body frame in the world frame;

    std::vector<Vector3d> y_measure_imu_2_foot_stack; // Leg kinematics stacks
    std::vector<Vector3d> lidar_measure_imu_2_foot_stack; // spatial lidar measurements stacks
    std::vector<double> contact_input_stack; // Contact stacks; 0, no contact, 1, contact

    //---------------------------------------------------------------
    // vo params
    std::vector<int> vo_insert_idx_stack;
    std::vector<int> vo_insert_discrete_time_stack;

    Vector3d vo_pose_body_pre_2_body_ = Vector3d::Zero();
    bool vo_to_be_processed_flag = false;
    Matrix3d R_vo_sb_pre_ = Matrix3d::Zero();

    std::vector<double> vo_time_stack;
    std::vector<int> vo_sychron_time_stack;
    std::vector<int> vo_sychron_discrete_time_stack;
    std::vector<int> vo_matches_stack;
    
    std::vector<Vector3d> vo_pose_body_pre_2_body_stack;
    std::vector<Matrix3d> R_vo_sb_rotation_stack; // Spatial orientation stacks; R_sb, body frame in the world frame when got camera;

private:
    //---------------------------------------------------------------
    // Dynamics cost terms
    // 0.5 * ||w_i||^2 _{Q_dyn}
    // A_dyn x_i - x_{i+1} - b_dyn -w_i = 0
    int dim_state = 12;

    SparseMatrix<double> Identity_dyn;

    // A_dyn:
    // [    I   dt* I   0   - 0.5 * dt^2 * R_sb;  ]
    // [    0   I       0   - dt * R_sb        ;  ]
    // [    0   0       I   0                  ;  ]
    // [    0   0       0   I                  ;  ]
    SparseMatrix<double> A_dyn;

    // b_dyn:
    // [    - 0.5 * dt^2 * accel_s; ]   p_s
    // [    - dt * accel_s        ; ]   v_s
    // [    0                     ; ]   p_foot_s
    // [    0                     ; ]   accel_bias_b
    VectorXd b_dyn = VectorXd::Zero(dim_state);

    // Q_dyn:
    // If no contact; Q_foot = 0 * R_sb* Q_foot_slide * R_sb'
    // [    Q_p    0    0                                          0;       ]
    // [    0      Q_v  0                                          0;       ]
    // [    0      0    contact * R_sb* Q_foot_slide * R_sb'       0;       ]
    // [    0      0    0                                          Q_accel; ]
    SparseMatrix<double> Q_dyn; // Q: gains matrix

    Matrix3d Q_foot_slide = Matrix3d::Zero(); // Gain process foot, small for little slippery
    Matrix3d Q_accel_bias = Matrix3d::Zero(); // Gain process accel_bias

    Matrix3d C_accel = Matrix3d::Zero();
    Matrix3d C_accel_contact = Matrix3d::Zero();
    Matrix3d C_accel_impact = Matrix3d::Zero();

    Matrix3d C_p = Matrix3d::Zero();

    //---------------------------------------------------------------
    // Prior cost term
    // 0.5 * || x_0 - x_prior_0 ||^2 _{Q_prior_0}

    // x_prior:
    // [    0                       ;   ]   p_s
    // [    0                       ;   ]   v_s
    // [    R_sb * leg_imu_2_foot_b ;   ]   p_foot_s
    // [    0                       ;   ]   accel_bias_b
    VectorXd x_prior = VectorXd::Zero(dim_state);

    // Q_prior_0:
    //   [  Q_p_init  0          0             0                ;   ]
    //   [  0         Q_v_init   0             0                ;   ]
    //   [  0         0          Q_foot_init   0                ;   ]
    //   [  0         0          0             Q_accel_bias_init;   ]
    SparseMatrix<double> Q_prior; // Gain prior

private:
    //---------------------------------------------------------------
    // Measurement cost&constraints term
    // 0.5 * || v_i ||^2 _{Q_meas_}
    // A_meas x_i - b_meas - v_i = 0
    double infinite = 1e6;

    int dim_meas = 4;

    SparseMatrix<double> Identity_meas;

    // A_meas:
    //   [  [0 0 1]  0   0   0; ]
    SparseMatrix<double> A_meas;

    // b_meas:
    // [    R_sb * leg_imu_2_foot_b(2)  ;   ]
    VectorXd b_meas = VectorXd::Zero(dim_meas);

    // Q_meas:
    //  [   R_sb * ( C_lidar )^{-1} * R_sb';    ]
    SparseMatrix<double> Q_meas; // Q_meas = C_meas^{-1}

    // C_meas:
    //  [   R_sb * C_lidar * R_sb';    ]
    Matrix3d C_lidar = Matrix3d::Zero(); // Covariance measurement encoder; rads

    //---------------------------------------------------------------
    // Camera cost&constraints term
    // A_cam_pre * x_k - A_cam_now * x_k+1 - vcam_k = b_cam_k
    int dim_cam = 3;

    SparseMatrix<double> Identity_cam_meas;

    // A_cam_pre:
    //   [  I  0   0   0;];
    SparseMatrix<double> A_cam_pre;
    // A_cam_now:
    //   [  I  0   0   0;];
    SparseMatrix<double> A_cam_now;

    // b_cam_pre:
    // [    - R_sb_pre * vo_realtive_pose  ;   ]
    VectorXd b_cam = VectorXd::Zero(dim_cam);

    // Q_cam_pre:
    //  [   R_sb_pre * (C_cam)^{-1} * R_sb_pre'  ;  ]
    SparseMatrix<double> Q_cam;
    Matrix3d Q_vo_pose = Matrix3d::Zero();

public:
    Matrix3d R_sb_;

    //---------------------------------------------------------------
    // Moving horizon estimation
    VectorXd x_MHE = VectorXd::Zero(21);
    VectorXd x_MHE_traj = VectorXd::Zero(141);
    Vector3d v_sb_ = Vector3d::Zero();

    //---------------------------------------------------------------
    // Kalman filter
    VectorXd x_KF_ = VectorXd::Zero(dim_state);
    MatrixXd C_KF_;
    MatrixXd K_KF_;

    Vector3d x_body_accmulate = Vector3d::Zero();
    Vector3d x_body_interpolate_accmulate = Vector3d::Zero();

private:
    void InitializeMHE();
    void UpdateMHE(int discrete_time);
    void GetMeasurement(int discrete_time);
    void AssembleImage(int discrete_time);

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