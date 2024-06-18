#include "../include/DecentralEst.hpp"
#include <fstream>

//  comments with math
//  debugging break using the ptr with the matrix

//  eigen operater = vs <<
// eigen block
//  add ros msg typr to eigen lib
//  ros2 params shorter
//  wrapper

DecentralizedEstimation::DecentralizedEstimation()
{
}

// Set the Estimation parameters and QP solver
void DecentralizedEstimation::initialize(std::shared_ptr<robot_store> sub, std::shared_ptr<robot_params> params)
{
    robot_sub_ptr_ = sub;
    params_ptr_ = params;

    est_type_ = params_ptr_->est_type_;
    N_ = params_ptr_->N_;
    mhe_qp_.setHorizon(N_, dim_state_, dim_meas_);

    gravity_ << 0, 0, -9.81;

    //---------------------------------------------------------------
    // Gain setup from params; Q: gains, inv of covariance; C: covariance
    infinite_ = params_ptr_->foot_swing_std_;

    StdVec2CovMat(params_ptr_->p_process_std_, C_p_);
    StdVec2CovMat(params_ptr_->accel_input_std_, C_accel_);
    StdVec2CovMat(params_ptr_->accel_bias_std_, C_accel_bias_);
    StdVec2CovMat(params_ptr_->foothold_slide_std_, C_foot_slide_);

    StdVec2GainMat(params_ptr_->accel_bias_std_, Q_accel_bias_);
    StdVec2GainMat(params_ptr_->foothold_slide_std_, Q_foot_slide_);

    // Declair the sparse matrix to the proper size
    Q_prior_.resize(dim_state_, dim_state_);
    Q_prior_.setZero();

    Identity_dyn_.resize(dim_state_, dim_state_);
    Identity_dyn_.setIdentity();
    A_dyn_.resize(dim_state_, dim_state_);
    A_dyn_.setIdentity();
    Q_dyn_.resize(dim_state_, dim_state_);
    Q_dyn_.setZero();

    Identity_meas_.resize(dim_meas_, dim_meas_);
    Identity_meas_.setIdentity();
    A_meas_.resize(dim_meas_, dim_state_);
    A_meas_.setZero();
    Q_meas_.resize(dim_meas_, dim_meas_);
    Q_meas_.setZero();

    // Const Matrix setup
    // A_meas_
    //   [  -I  0   I   0;
    //      -I  0   I   0;  ]
    MatrixXd A_meas_dense = MatrixXd::Zero(dim_meas_, dim_state_);
    A_meas_dense.block<3, 3>(0, 0) << -Matrix3d::Identity();
    A_meas_dense.block<3, 3>(0, 6) << Matrix3d::Identity();
    A_meas_dense.block<3, 3>(3, 0) << -Matrix3d::Identity();
    A_meas_dense.block<3, 3>(3, 9) << Matrix3d::Identity();
    EigenUtils::SparseMatrixBlockAsignFromDense(A_meas_, 0, 0, A_meas_dense);

    //---------------------------------------------------------------
    // Initialize the estimator
    switch (est_type_)
    {
    case 0:
        InitializeMHE();
        break;
    case 1:
        InitializeKF();
        UpdateKF();
        break;
    default:
        std::cout << est_type_ + " not a valid estimation type." << std::endl;
        break;
    }
}

void DecentralizedEstimation::update(int T)
{

    switch (est_type_)
    {
    case 0:
    {

        UpdateMHE(T);
        // std::cout << "updateMHE to " + std::to_string(T) << std::endl;

        if (T >= N_)
        {
            mhe_qp_.marginalizeQP(T - N_); // the marginalization is independent of update() and assemble_image()
        }
        // tic("init");
        mhe_qp_.initQP(T); // update the osqp, note to keep the sparsity structure
        // toc("init");

        // tic("solve");
        mhe_qp_.solveQP();
        // toc("solve");

        std::string var_name = "x_" + std::to_string(T);
        mhe_qp_.getsolution(T);

        x_MHE_ = mhe_qp_.x_solution_now;

        break;
    }
    case 1:
    {
        UpdateKF();
        break;
    }
    }
}

void DecentralizedEstimation::InitializeMHE()
{

    //---------------------------------------------------------------
    // OSQP configurate
    mhe_qp_.osqp.settings()->setWarmStart(true);
    mhe_qp_.osqp.settings()->setAdaptiveRho(params_ptr_->adaptRho_);
    mhe_qp_.osqp.settings()->setVerbosity(params_ptr_->verbose_);
    mhe_qp_.osqp.settings()->setPolish(params_ptr_->polish_);
    mhe_qp_.osqp.settings()->setMaxIteration(params_ptr_->maxQPIter_);
    mhe_qp_.osqp.settings()->setRho(params_ptr_->rho_);
    mhe_qp_.osqp.settings()->setAlpha(params_ptr_->alpha_);
    mhe_qp_.osqp.settings()->setDelta(params_ptr_->delta_);
    mhe_qp_.osqp.settings()->setSigma(params_ptr_->sigma_);
    mhe_qp_.osqp.settings()->setRelativeTolerance(params_ptr_->realtiveTol_);
    mhe_qp_.osqp.settings()->setAbsoluteTolerance(params_ptr_->absTol_);
    mhe_qp_.osqp.settings()->setPrimalInfeasibilityTollerance(params_ptr_->primTol_);
    mhe_qp_.osqp.settings()->setDualInfeasibilityTollerance(params_ptr_->dualTol_);
    mhe_qp_.osqp.settings()->setTimeLimit(params_ptr_->timeLimit_);

    //---------------------------------------------------------------
    // Prior cost at 0
    GetMeasurement(0);

    Matrix3d R_sb = R_input_rotation_stack_.back();

    b_meas_.segment<3>(0) << R_sb * p_left_imu_2_foot_stack_.back();
    b_meas_.segment<3>(3) << R_sb * p_right_imu_2_foot_stack_.back();

    // Q_meas_:
    //  [   R_sb * cov_foot^{-1} * R_sb'    ;   ]
    MatrixXd Q_meas_dense = MatrixXd::Zero(6, 6);
    Q_meas_dense.block<3, 3>(0, 0) = R_sb *
                                     cov_left_input_stack_.back().inverse() *
                                     R_sb.transpose();
    Q_meas_dense.block<3, 3>(3, 3) = R_sb *
                                     cov_right_input_stack_.back().inverse() *
                                     R_sb.transpose();
    Q_meas_.setZero();

    EigenUtils::SparseMatrixBlockAsignFromDense(Q_meas_, 0, 0, Q_meas_dense);

    // x_prior_:
    // [    0                       ;   ]   p_s
    // [    0                       ;   ]   v_s
    // [    R_sb * p_leg_imu_2_foot_b ;   ]   p_foot_s
    // [    0                       ;   ]   accel_bias_b
    x_prior_.segment<3>(0) << 0.0, 0.0, 0.0; // Unobservable
    x_prior_.segment<3>(3) << 0.0, 0.0, 0.0;
    x_prior_.segment<6>(6) = b_meas_;
    x_prior_.segment<3>(12) << 0.0, 0.0, 0.0;

    Matrix3d Q_pint = Matrix3d::Zero();
    Matrix3d Q_vint = Matrix3d::Zero();

    StdVec2GainMat(params_ptr_->p_init_std_, Q_pint);
    StdVec2GainMat(params_ptr_->v_init_std_, Q_vint);

    // Q_prior_:
    //   [  Q_p_init  0          0             0                ;   ]
    //   [  0         Q_v_init   0             0                ;   ]
    //   [  0         0          Q_foot_init   0                ;   ]
    //   [  0         0          0             Q_accel_bias_init;   ]
    Q_prior_.setZero();
    EigenUtils::SparseMatrixBlockAsignFromDense(Q_prior_, 0, 0, Q_pint);
    EigenUtils::SparseMatrixBlockAsignFromDense(Q_prior_, 3, 3, Q_vint);
    EigenUtils::SparseMatrixBlockAsignFromDense(Q_prior_, 6, 6, 1 / std::pow(params_ptr_->foot_init_std_, 2) * Matrix3d::Identity());
    EigenUtils::SparseMatrixBlockAsignFromDense(Q_prior_, 9, 9, 1 / std::pow(params_ptr_->foot_init_std_, 2) * Matrix3d::Identity());
    EigenUtils::SparseMatrixBlockAsignFromDense(Q_prior_, 12, 12, 1 / std::pow(params_ptr_->accel_bias_init_std_, 2) * Matrix3d::Identity());

    // || x_0 - x_prior_ ||^2_{Q_prior_}
    mhe_qp_.addVariable("x_0", dim_state_);
    mhe_qp_.addCost("Prior_0", x_prior_, Q_prior_);
    mhe_qp_.addCostDependency("Prior_0", "x_0", Identity_dyn_);

    // A_meas_ x_i - v_{i} = b_meas_
    mhe_qp_.addVariable("v_0", dim_meas_);
    mhe_qp_.addConstraints("Measurement_0", b_meas_, b_meas_);
    mhe_qp_.addConstraintDependency("Measurement_0", "x_0", A_meas_);
    mhe_qp_.addConstraintDependency("Measurement_0", "v_0", -Identity_meas_);

    // || v_i ||^2_{Q_meas_}
    mhe_qp_.addCost("Measurement_0", VectorXd::Zero(dim_meas_), Q_meas_);
    mhe_qp_.addCostDependency("Measurement_0", "v_0", Identity_meas_);

    mhe_qp_.updateQP(0);
}

void DecentralizedEstimation::UpdateMHE(int T)
{
    //---------------------------------------------------------------
    // Dynamic cost at T-1
    // 0.5 * || x_T - f( x_{T-1},u_{T-1} ) ||^2_{Q_{T-1}} => 0.5 * || Adyn x_{T-1} - x_T - b_dyn_ ||^2_{Q_{T-1}}
    std::string T_pre_string = std::to_string(T - 1);
    std::string T_string = std::to_string(T);
    std::string X_T_pre_string = "x_" + T_pre_string;
    std::string X_T_string = "x_" + T_string;

    std::string W_T_pre_string = "w_" + T_pre_string;

    std::string Dyn_string = "Dynamic_" + T_pre_string;

    mhe_qp_.addVariable(W_T_pre_string, dim_state_);
    mhe_qp_.addVariable(X_T_string, dim_state_);

    Matrix3d R_sb = R_input_rotation_stack_.back();
    Vector3d accel_s = accel_s_input_stack_.back();
    double dt = dt_stack_.back();

    // b_dyn_:
    // [    - 0.5 * dt^2 * accel_s; ]   p_s
    // [    - dt * accel_s        ; ]   v_s
    // [    0                     ; ]   p_foot_s
    // [    0                     ; ]   accel_bias_b
    b_dyn_.segment(0, 3) << -dt * dt / 2 * accel_s;
    b_dyn_.segment(3, 3) << -dt * accel_s;

    // A_dyn_:
    // [    I   dt* I   0   - 0.5 * dt^2 * R_sb;  ]
    // [    0   I       0   - dt * R_sb        ;  ]
    // [    0   0       I   0                  ;  ]
    // [    0   0       0   I                  ;  ]
    A_dyn_.setIdentity();
    EigenUtils::SparseMatrixBlockAsignFromDense(A_dyn_, 0, 3, dt * Matrix3d::Identity());
    EigenUtils::SparseMatrixBlockAsignFromDense(A_dyn_, 0, 12, -dt * dt / 2 * R_sb);
    EigenUtils::SparseMatrixBlockAsignFromDense(A_dyn_, 3, 12, -dt * R_sb);

    // Q_dyn_:
    // (G_dyn * diag[C_p_, C_accel_, C_foot_slide_, C_accel_bias_  ] * G_dyn').inverse()
    // C_slide = infinite_, if contact_foot = false
    // G_dyn:
    // [  R_sb * dt          -0.5 * R_sb *dt^2   0           0;      ]
    // [  0                  -R_sb *dt           0           0;      ]
    // [  0                  0                   R_sb * dt   0;      ]
    // [  0                  0                   0           I * dt; ]
    Q_dyn_.setZero();

    MatrixXd G_dyn_pv = MatrixXd::Zero(6, 6);
    G_dyn_pv.block<3, 3>(0, 0) = R_sb * dt;
    G_dyn_pv.block<3, 3>(0, 3) = 0.5 * R_sb * dt * dt;
    G_dyn_pv.block<3, 3>(3, 3) = R_sb * dt;

    MatrixXd C_dyn_pv = MatrixXd::Zero(6, 6);
    C_dyn_pv.block<3, 3>(0, 0) = C_p_;
    C_dyn_pv.block<3, 3>(3, 3) = C_accel_;

    MatrixXd Q_dyn_pv = (G_dyn_pv * C_dyn_pv * G_dyn_pv.transpose()).inverse();

    EigenUtils::SparseMatrixBlockAsignFromDense(Q_dyn_, 0, 0, Q_dyn_pv);

    // left foot covariance
    if (contact_left_input_stack_.back()) // if contact at foot_idx, fixed the stance foot position using large Q_foot_slide_
    {
        EigenUtils::SparseMatrixBlockAsignFromDense(Q_dyn_,
                                                    6, 6,
                                                    1 / (dt * dt) *
                                                        R_sb * Q_foot_slide_ * R_sb.transpose());
    }
    else // if no contact at foot_idx, putting small gains on stance foot position
    {
        EigenUtils::SparseMatrixBlockAsignFromDense(Q_dyn_,
                                                    6, 6,
                                                    1 / (dt * dt) / infinite_ * Matrix3d::Identity());
    }
    // right foot covariance
    if (contact_right_input_stack_.back()) // if contact at foot_idx, fixed the stance foot position using large Q_foot_slide_
    {
        EigenUtils::SparseMatrixBlockAsignFromDense(Q_dyn_,
                                                    9, 9,
                                                    1 / (dt * dt) *
                                                        R_sb * Q_foot_slide_ * R_sb.transpose());
    }
    else // if no contact at foot_idx, putting small gains on stance foot position
    {
        EigenUtils::SparseMatrixBlockAsignFromDense(Q_dyn_,
                                                    9, 9,
                                                    1 / (dt * dt) / infinite_ * Matrix3d::Identity());
    }

    EigenUtils::SparseMatrixBlockAsignFromDense(Q_dyn_,
                                                12, 12,
                                                1 / (dt * dt) * Q_accel_bias_);

    // A_dyn_ x_i - x_{i+1} - w_{i} = b_dyn_
    mhe_qp_.addConstraints(Dyn_string, b_dyn_, b_dyn_);
    mhe_qp_.addConstraintDependency(Dyn_string, X_T_pre_string, A_dyn_);
    mhe_qp_.addConstraintDependency(Dyn_string, X_T_string, -Identity_dyn_);
    mhe_qp_.addConstraintDependency(Dyn_string, W_T_pre_string, -Identity_dyn_);

    // 0.5 * || w_{i} ||^2_{Q_dyn_}
    mhe_qp_.addCost(Dyn_string, VectorXd::Zero(dim_state_), Q_dyn_);
    mhe_qp_.addCostDependency(Dyn_string, W_T_pre_string, Identity_dyn_);

    //---------------------------------------------------------------
    // Measurement cost at T
    GetMeasurement(T);

    std::string V_T_string = "v_" + T_string;
    std::string Meas_string = "Measurement_" + T_string;
    mhe_qp_.addVariable(V_T_string, dim_meas_);

    R_sb = R_input_rotation_stack_.back();

    // b_meas_:
    b_meas_.segment<3>(0) << R_sb * p_left_imu_2_foot_stack_.back();
    b_meas_.segment<3>(3) << R_sb * p_right_imu_2_foot_stack_.back();

    // Q_meas_:
    //  [   R_sb * cov_foot^{-1} * R_sb'    ;   ]
    Q_meas_.setZero();
    MatrixXd Q_meas_dense = MatrixXd::Zero(6, 6);
    Q_meas_dense.block<3, 3>(0, 0) = R_sb *
                                     cov_left_input_stack_.back().inverse() *
                                     R_sb.transpose();
    Q_meas_dense.block<3, 3>(3, 3) = R_sb *
                                     cov_left_input_stack_.back().inverse() *
                                     R_sb.transpose();
    EigenUtils::SparseMatrixBlockAsignFromDense(Q_meas_, 0, 0, Q_meas_dense);

    // A_meas_ x_i - v_{i} = b_meas_
    mhe_qp_.addConstraints(Meas_string, b_meas_, b_meas_);
    mhe_qp_.addConstraintDependency(Meas_string, X_T_string, A_meas_);
    mhe_qp_.addConstraintDependency(Meas_string, V_T_string, -Identity_meas_);

    // 0.5 * || v_{i} ||^2_{Q_meas_}
    mhe_qp_.addCost(Meas_string, VectorXd::Zero(dim_meas_), Q_meas_);
    mhe_qp_.addCostDependency(Meas_string, V_T_string, Identity_meas_);

    mhe_qp_.updateQP(T);
}

// Tested
void DecentralizedEstimation::GetMeasurement(int T)
{
    // get current measurement from sub_ptr

    dt_ = robot_sub_ptr_->delta_t_;

    R_sb_ = robot_sub_ptr_->Rotation_; // Estimated rotation from Orientation EKF

    Vector3d accel_b = robot_sub_ptr_->accel_b_;
    accel_s_ = R_sb_ * accel_b + gravity_; // gravity free acceleration in world frame

    contact_left_ = robot_sub_ptr_->contact_left_;
    contact_right_ = robot_sub_ptr_->contact_right_;
    p_left_ = robot_sub_ptr_->Kinematics_left_.block<3, 1>(0, 3);
    p_right_ = robot_sub_ptr_->Kinematics_right_.block<3, 1>(0, 3);
    Cov_left_ = robot_sub_ptr_->Cov_left_.block<3, 3>(3, 3);
    Cov_right_ = robot_sub_ptr_->Cov_right_.block<3, 3>(3, 3);

    dt_stack_.push_back(dt_);

    accel_s_input_stack_.push_back(accel_s_);
    R_input_rotation_stack_.push_back(R_sb_);

    contact_left_input_stack_.push_back(contact_left_);
    contact_right_input_stack_.push_back(contact_right_);

    p_left_imu_2_foot_stack_.push_back(p_left_);
    p_right_imu_2_foot_stack_.push_back(p_right_);

    cov_left_input_stack_.push_back(Cov_left_);
    cov_right_input_stack_.push_back(Cov_right_);

    // only keep the previous N + 1 measurements, but the storage size is arbitary since no stack.front() is used
    // discrete time now: T
    // discrete time optimization window start: T-N
    if (int(accel_s_input_stack_.size()) > N_ + 1) // should be n+1 to store the measments in the current optimization window
    {
        dt_stack_.erase(dt_stack_.begin());
        R_input_rotation_stack_.erase(R_input_rotation_stack_.begin());
        contact_left_input_stack_.erase(contact_left_input_stack_.begin());
        contact_right_input_stack_.erase(contact_right_input_stack_.begin());
        p_left_imu_2_foot_stack_.erase(p_left_imu_2_foot_stack_.begin());
        p_right_imu_2_foot_stack_.erase(p_right_imu_2_foot_stack_.begin());
        cov_left_input_stack_.erase(cov_left_input_stack_.begin());
        cov_right_input_stack_.erase(cov_right_input_stack_.begin());
    }
}

void DecentralizedEstimation::InitializeKF()
{
    GetMeasurement(0);

    Matrix3d R_sb = R_input_rotation_stack_.back();

    b_meas_.segment<3>(0) << R_sb * p_left_imu_2_foot_stack_.back();
    b_meas_.segment<3>(3) << R_sb * p_right_imu_2_foot_stack_.back();

    // x_prior_:
    // [    0                       ;   ]   p_s
    // [    0                       ;   ]   v_s
    // [    R_sb * p_leg_imu_2_foot_b ;   ]   p_foot_s
    // [    0                       ;   ]   accel_bias_b
    x_prior_.segment<3>(0) << 0.0, 0.0, 0.0;
    x_prior_.segment<3>(3) << 0.0, 0.0, 0.0;
    x_prior_.segment<6>(6) = b_meas_;
    x_prior_.segment<3>(12) << 0.0, 0.0, 0.0;

    x_KF_ = x_prior_;

    // KF prior
    MatrixXd C_prior = MatrixXd::Zero(dim_state_, dim_state_);

    Matrix3d C_pint = Matrix3d::Zero();
    Matrix3d C_vint = Matrix3d::Zero();

    StdVec2CovMat(params_ptr_->p_init_std_, C_pint);
    StdVec2CovMat(params_ptr_->v_init_std_, C_vint);

    C_prior.block<3, 3>(0, 0) = C_pint;
    C_prior.block<3, 3>(3, 3) = C_vint;
    C_prior.block<3, 3>(6, 6) = std::pow(params_ptr_->foot_init_std_, 2) * MatrixXd::Identity(3, 3);
    C_prior.block<3, 3>(9, 9) = std::pow(params_ptr_->foot_init_std_, 2) * MatrixXd::Identity(3, 3);
    C_prior.block<3, 3>(9, 9) = std::pow(params_ptr_->accel_bias_init_std_, 2) * Matrix3d::Identity();

    C_KF_ = C_prior;
}

void DecentralizedEstimation::UpdateKF()
{
    //---------------------------------------------------------------
    // KF measurement update
    GetMeasurement(0);

    Matrix3d R_sb = R_input_rotation_stack_.back();

    b_meas_.segment<3>(0) << R_sb * p_left_imu_2_foot_stack_.back();
    b_meas_.segment<3>(3) << R_sb * p_right_imu_2_foot_stack_.back();

    // C_meas
    //  [   R_sb * C_kinematic  * R_sb';    ]
    MatrixXd C_meas = MatrixXd::Zero(dim_meas_, dim_meas_);
    C_meas.block<3, 3>(0, 0) = R_sb *
                               cov_left_input_stack_.back() *
                               R_sb.transpose();
    C_meas.block<3, 3>(3, 3) = R_sb *
                               cov_right_input_stack_.back() *
                               R_sb.transpose();

    // KF measurement correction
    K_KF_ = C_KF_ * A_meas_.transpose() * (A_meas_ * C_KF_ * A_meas_.transpose() + C_meas).inverse();
    x_KF_ = x_KF_ + K_KF_ * (b_meas_ - A_meas_ * x_KF_);
    C_KF_ = (MatrixXd::Identity(dim_state_, dim_state_) - K_KF_ * A_meas_) * C_KF_;

    //---------------------------------------------------------------
    // KF prediction update
    Vector3d accel_s = accel_s_input_stack_.back();
    double dt = dt_stack_.back();
    // b_dyn_:
    // [    - 0.5 * dt^2 * accel_s; ]   p_s
    // [    - dt * accel_s        ; ]   v_s
    // [    0                     ; ]   p_foot_s
    // [    0                     ; ]   accel_bias_b
    b_dyn_.segment(0, 3) << -0.5 * dt * dt * accel_s;
    b_dyn_.segment(3, 3) << -dt * accel_s;

    // A_dyn_:
    // [    I   dt* I   0   - 0.5 * dt^2 * R_sb;  ]
    // [    0   I       0   - dt * R_sb        ;  ]
    // [    0   0       I   0                  ;  ]
    // [    0   0       0   I                  ;  ]

    // p_{i+1} = p_i + dt * v_i + 0.5 * dt^2 * accel_s - 0.5 * dt^2 * R_sb * accel_b_bias + w_p;
    // v_{i+1} = v_i + dt * accel_s - dt * R_sb * accel_b_bias + w_v;
    // p_foot_{i+1} = p_foot_{i} + w_foot
    // accel_b_bias_{i+1} = accel_b_bias_{i} + w_a_bias
    A_dyn_.setIdentity();
    EigenUtils::SparseMatrixBlockAsignFromDense(A_dyn_, 0, 3, dt * Matrix3d::Identity());
    EigenUtils::SparseMatrixBlockAsignFromDense(A_dyn_, 0, 12, -dt * dt / 2 * R_sb);
    EigenUtils::SparseMatrixBlockAsignFromDense(A_dyn_, 3, 12, -dt * R_sb);

    // KF prediction propagate
    x_KF_ = A_dyn_ * x_KF_ - b_dyn_;

    double infinite_ = params_ptr_->foot_swing_std_;

    MatrixXd C_dyn = MatrixXd::Zero(dim_state_, dim_state_);

    // C_dyn:
    // G_dyn * diag[ C_p_, C_accel_, C_slide, C_accel_bias_  ] * G_dyn'
    // G_dyn: Uncertainty from input
    // [  R_sb * dt          -0.5 * R_sb *dt^2   0           0;      ]
    // [  0                  -R_sb *dt           0           0;      ]
    // [  0                  0                   R_sb * dt   0;      ]
    // [  0                  0                   0           I * dt; ]
    // C_slide = infinite_, if contact(feet) = false

    MatrixXd G_dyn = MatrixXd::Zero(dim_state_, dim_state_);
    G_dyn.block<3, 3>(0, 0) = R_sb * dt;
    G_dyn.block<3, 3>(0, 3) = -0.5 * R_sb * dt * dt;
    G_dyn.block<3, 3>(3, 3) = -R_sb * dt;
    G_dyn.block<3, 3>(6, 6) = R_sb * dt;
    G_dyn.block<3, 3>(9, 9) = R_sb * dt;
    G_dyn.block<3, 3>(12, 12) = Matrix3d::Identity() * dt;

    MatrixXd C_input = MatrixXd::Zero(dim_state_, dim_state_);
    C_input.block<3, 3>(0, 0) = C_p_;
    C_input.block<3, 3>(3, 3) = C_accel_;
    C_input.block<3, 3>(6, 6) = C_foot_slide_;
    C_input.block<3, 3>(9, 9) = C_foot_slide_;

    C_input.block<3, 3>(12, 12) = C_accel_bias_;

    if (!(contact_left_input_stack_.back()))
    {
        C_input.block<3, 3>(6, 6) << infinite_ * Matrix3d::Identity();
    }
    if (!(contact_right_input_stack_.back()))
    {
        C_input.block<3, 3>(9, 9) << infinite_ * Matrix3d::Identity();
    }

    C_dyn = G_dyn * C_input * G_dyn.transpose();

    C_KF_ = A_dyn_ * C_KF_ * A_dyn_.transpose() + C_dyn;
}

void DecentralizedEstimation::reset()
{

    mhe_qp_.resetQP();
}

void DecentralizedEstimation::StdVec2CovMat(const std::vector<double> &std, Matrix3d &Cov)
{
    Cov.diagonal() << std::pow(std[0], 2),
        std::pow(std[1], 2),
        std::pow(std[2], 2);
}

void DecentralizedEstimation::StdVec2GainMat(const std::vector<double> &std, Matrix3d &Gain)
{
    Gain.diagonal() << 1 / std::pow(std[0], 2),
        1 / std::pow(std[1], 2),
        1 / std::pow(std[2], 2);
}

void DecentralizedEstimation::Log2txt(const MatrixXd matrix, std::string filename)
{
    IOFormat CleanFmt(10, 0, ", ", "\n", "[", "]");
    std::string path = "/home/jkang/mhe_ros/logtxt/";
    std::ofstream outfile(path + filename + ".txt");
    outfile << matrix.format(CleanFmt);
    outfile.close();
}

void DecentralizedEstimation::tic(std::string str, int mode)
{
    static std::chrono::_V2::system_clock::time_point t_start;

    if (mode == 0)
        t_start = std::chrono::high_resolution_clock::now();
    else
    {
        auto t_end = std::chrono::high_resolution_clock::now();
        std::cout << str + " elapsed time: " << (t_end - t_start).count() * 1E-9 << " seconds\n";
    }
}

void DecentralizedEstimation::toc(std::string str) { tic(str, 1); }
