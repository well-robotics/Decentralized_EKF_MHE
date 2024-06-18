#include "../include/DecentralEst.hpp"
#include <fstream>

DecentralizedEstimation::DecentralizedEstimation()
{
}

// Set the Estimation parameters and QP solver
void DecentralizedEstimation::initialize(std::shared_ptr<robot_store> sub, std::shared_ptr<robot_params> params)
{
    robot_sub_ptr_ = sub;
    params_ptr_ = params;

    est_type_ = params_ptr_->est_type_;
    dt_ = 1.0 / params_ptr_->rate_;
    N_ = params_ptr_->N_;
    mhe_qp_.setHorizon(N_, dim_state_, dim_meas_, dim_cam_);

    contact_effort_theshold_ = params_ptr_->contact_effort_theshold_;
    gravity_ << 0, 0, -9.81;
    p_ib_ << params_ptr_->p_ib_[0], params_ptr_->p_ib_[1], params_ptr_->p_ib_[2];
    //---------------------------------------------------------------
    // Gain setup from params; Q: gains, inv of covariance; C: covariance
    infinite_ = params_ptr_->foot_swing_std_; // allows for foot state update

    StdVec2CovMat(params_ptr_->p_process_std_, C_p_);
    StdVec2CovMat(params_ptr_->accel_input_std_, C_accel_);
    StdVec2CovMat(params_ptr_->accel_bias_std_, C_accel_bias_);
    StdVec2CovMat(params_ptr_->joint_position_std_, C_encoder_position_);
    StdVec2CovMat(params_ptr_->joint_velocity_std_, C_encoder_velocity_);
    StdVec2CovMat(params_ptr_->gyro_input_std_, C_gyro_);
    StdVec2CovMat(params_ptr_->foothold_slide_std_, C_foot_slide_);

    StdVec2GainMat(params_ptr_->accel_bias_std_, Q_accel_bias_);
    StdVec2GainMat(params_ptr_->vo_p_std_, Q_vo_p_);

    // Declare the sparse matrix to the proper size
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

    Identity_cam_meas_.resize(dim_cam_, dim_cam_);
    Identity_cam_meas_.setIdentity();
    A_cam_pre_.resize(dim_cam_, dim_state_);
    A_cam_pre_.setZero();
    A_cam_now_.resize(dim_cam_, dim_state_);
    A_cam_now_.setZero();
    Q_cam_.resize(dim_cam_, dim_cam_);
    Q_cam_.setZero();

    // Const Matrix setup
    //---------------------------------------------------------------
    // A_meas_, *x:
    //   [  0   I   0;  ]
    MatrixXd A_meas_dense = MatrixXd::Zero(dim_meas_, dim_state_);
    A_meas_dense.block<3, 3>(0, 3) << Matrix3d::Identity();
    A_meas_dense.block<3, 3>(3, 3) << Matrix3d::Identity();
    A_meas_dense.block<3, 3>(6, 3) << Matrix3d::Identity();
    A_meas_dense.block<3, 3>(9, 3) << Matrix3d::Identity();
    EigenUtils::SparseMatrixBlockAsignFromDense(A_meas_, 0, 0, A_meas_dense);

    // A_cam_pre_/A_cam_now_, *x:
    //   diag[  I   0   0;  ]
    A_cam_pre_.coeffRef(0, 0) = 1.0;
    A_cam_pre_.coeffRef(1, 1) = 1.0;
    A_cam_pre_.coeffRef(2, 2) = 1.0;
    A_cam_now_.coeffRef(0, 0) = 1.0;
    A_cam_now_.coeffRef(1, 1) = 1.0;
    A_cam_now_.coeffRef(2, 2) = 1.0;

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

        if (vo_to_be_processed_flag_)
        {
            UpdateVOConstraints(T);
            vo_to_be_processed_flag_ = false;
        }

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

        mhe_qp_.getsolution(T);

        x_MHE_ = mhe_qp_.x_solution_now;

        Vector3d p_imu_2_opti;
        p_imu_2_opti << 0.016041, 0.089061, 0.0579875;
        v_MHE_b_ = R_input_rotation_stack_.back() * (x_MHE_.segment<3>(3) + angular_b_input_stack_.back().cross(p_imu_2_opti));

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
    mhe_qp_.osqp.settings()->setPrimalInfeasibilityTolerance(params_ptr_->primTol_);
    mhe_qp_.osqp.settings()->setDualInfeasibilityTolerance(params_ptr_->dualTol_);
    mhe_qp_.osqp.settings()->setTimeLimit(params_ptr_->timeLimit_);

    //---------------------------------------------------------------
    // Prior cost at 0
    GetMeasurement(0);

    Matrix3d R_sb = R_input_rotation_stack_.back();

    // Q_meas_:
    //  [   R_sb * J * cov_encoder_position^{-1} * J' * R_sb' ];
    MatrixXd Q_meas_dense = MatrixXd::Zero(dim_meas_, dim_meas_);

    // b_meas_:
    //   [  -R_sb * J_foot * dq - R_sb omega.cross(kin_foot)   ];
    for (int i = 0; i < num_legs_; i++)
    {
        b_meas_.segment<3>(i * 3) = -R_sb * J_imu_2_foot_stack_.back().block<3, 3>(i * 3, 0) * joint_velocity_stack_.back().segment<3>(i * 3) -
                                   R_sb * angular_b_input_stack_.back().cross(p_imu_2_foot_stack_.back().block<3, 1>(i * 3, 0));
        if (contact_input_stack_.back()(i) == 0.0)
        {
            Q_meas_dense.block<3, 3>(i * 3, i * 3) << 1 / infinite_ * Matrix3d::Identity();
        }
        else
        {
            MatrixXd G_meas_i = MatrixXd::Zero(3, 9);

            MatrixXd C_meas_dense = MatrixXd::Zero(3, 3);

            G_meas_i.block<3, 3>(0, 0) = -J_imu_2_foot_stack_.back().block<3, 3>(i * 3, 0);

            Matrix3d omega_skew = Matrix3d::Zero();
            EigenUtils::vector3dSkew(omega_skew, angular_b_input_stack_.back());
            // std::cout << -omega_skew * J_imu_2_foot_stack_.back().block<3, 3>(i * 3, 0) << std::endl;
            G_meas_i.block<3, 3>(0, 3) = -omega_skew * J_imu_2_foot_stack_.back().block<3, 3>(i * 3, 0);

            Matrix3d kin_skew = Matrix3d::Zero();
            EigenUtils::vector3dSkew(kin_skew, p_imu_2_foot_stack_.back().block<3, 1>(i * 3, 0));
            G_meas_i.block<3, 3>(0, 6) = kin_skew;

            MatrixXd C_meas_i = MatrixXd::Zero(9, 9);
            C_meas_i.block<3, 3>(0, 0) = C_encoder_velocity_;
            C_meas_i.block<3, 3>(3, 3) = C_encoder_position_;
            C_meas_i.block<3, 3>(6, 6) = C_gyro_;

            C_meas_dense = R_sb * G_meas_i * C_meas_i *
                           G_meas_i.transpose() * R_sb.transpose();

            // C_meas.block<3, 3>(i * 3, i * 3) = R_sb * G_meas_i * C_meas_i *
            //                                    G_meas_i.transpose() * R_sb.transpose();

            Q_meas_dense.block<3, 3>(i * 3, i * 3) = C_meas_dense.inverse();
        }
    }
    Q_meas_.setZero();
    EigenUtils::SparseMatrixBlockAsignFromDense(Q_meas_, 0, 0, Q_meas_dense);

    // x_prior:
    // [    0                       ;   ]   p_s
    // [    0                       ;   ]   v_s
    // [    0                       ;   ]   accel_bias_b
    x_prior_.segment<3>(0) << 0.0, 0.0, 0.0;
    x_prior_.segment<3>(3) << 0.0, 0.0, 0.0;
    x_prior_.segment<3>(6) << 0.0, 0.0, 0.00;

    Matrix3d Q_pint = Matrix3d::Zero();
    Matrix3d Q_vint = Matrix3d::Zero();

    StdVec2GainMat(params_ptr_->p_init_std_, Q_pint);
    StdVec2GainMat(params_ptr_->v_init_std_, Q_vint);

    // Q_prior_0:
    //   [  Q_p_init  0          0                ;   ]
    //   [  0         Q_v_init   0                ;   ]
    //   [  0         0          Q_accel_bias_init;   ]
    Q_prior_.setZero();
    EigenUtils::SparseMatrixBlockAsignFromDense(Q_prior_, 0, 0, Q_pint);
    EigenUtils::SparseMatrixBlockAsignFromDense(Q_prior_, 3, 3, Q_vint);
    EigenUtils::SparseMatrixBlockAsignFromDense(Q_prior_, 6, 6, 1 / std::pow(params_ptr_->accel_bias_init_std_, 2) * Matrix3d::Identity());

    std::cout << "x0----------------------------" << std::endl;
    std::cout << x_prior_ << std::endl;

    // || x_0 - x_prior||^2_{Q_prior_}
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
    // Dynamic cost & constraints at T-1
    // 0.5 * || x_T - f( x_{T-1},u_{T-1} ) ||^2_{Q_{T-1}} => 0.5 * || Adyn x_{T-1} - x_T - b_dyn_ ||^2_{Q_{T-1}}
    std::string T_pre_string = std::to_string(T - 1);
    std::string T_string = std::to_string(T);
    std::string X_T_pre_string = "x_" + T_pre_string;
    std::string X_T_string = "x_" + T_string;

    std::string W_T_pre_string = "w_" + T_pre_string;
    std::string Vcam_T_pre_string = "vcam_" + T_pre_string;
    std::string V_T_string = "v_" + T_string;

    std::string Dyn_string = "Dynamic_" + T_pre_string;
    std::string Cam_Meas_string = "VO_measurement_" + T_pre_string;
    std::string Meas_string = "Measurement_" + T_string;

    mhe_qp_.addVariable(W_T_pre_string, dim_state_);
    mhe_qp_.addVariable(Vcam_T_pre_string, dim_cam_);
    mhe_qp_.addVariable(X_T_string, dim_state_);
    mhe_qp_.addVariable(V_T_string, dim_meas_);

    Matrix3d R_sb = R_input_rotation_stack_.back();
    Vector3d accel_s = accel_s_input_stack_.back();
    double dt = dt_;

    // b_dyn_:
    // [    - 0.5 * dt^2 * accel_s; ]   p_s
    // [    - dt * accel_s        ; ]   v_s
    // [    0                     ; ]   accel_bias_b
    b_dyn_.segment(0, 3) << -dt * dt / 2 * accel_s;
    b_dyn_.segment(3, 3) << -dt * accel_s;

    // A_dyn_:
    // [    I   dt* I   - 0.5 * dt^2 * R_sb;  ]
    // [    0   I       - dt * R_sb        ;  ]
    // [    0   0       I                  ;  ]
    A_dyn_.setIdentity();
    EigenUtils::SparseMatrixBlockAsignFromDense(A_dyn_, 0, 3, dt * Matrix3d::Identity());
    EigenUtils::SparseMatrixBlockAsignFromDense(A_dyn_, 0, 6, -dt * dt / 2 * R_sb);
    EigenUtils::SparseMatrixBlockAsignFromDense(A_dyn_, 3, 6, -dt * R_sb);

    // Q_dyn_:
    // (G_dyn * diag[C_velocity, C_accel_, C_accel_bias_  ] * G_dyn').inverse()
    // G_dyn:
    // [  R_sb * dt          -0.5 * R_sb *dt^2   0;      ]
    // [  0                  -R_sb *dt           0           0;      ]
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

    EigenUtils::SparseMatrixBlockAsignFromDense(Q_dyn_,
                                                6, 6,
                                                1 / (dt * dt) * Q_accel_bias_);

    // A_dyn_ x_i - x_{i+1} - w_{i} = b_dyn_
    mhe_qp_.addConstraints(Dyn_string, b_dyn_, b_dyn_);
    mhe_qp_.addConstraintDependency(Dyn_string, W_T_pre_string, -Identity_dyn_);
    mhe_qp_.addConstraintDependency(Dyn_string, X_T_string, -Identity_dyn_);
    mhe_qp_.addConstraintDependency(Dyn_string, X_T_pre_string, A_dyn_);

    // 0.5 * || w_{i} ||^2 _{Q_dyn_}
    mhe_qp_.addCost(Dyn_string, VectorXd::Zero(dim_state_), Q_dyn_);
    mhe_qp_.addCostDependency(Dyn_string, W_T_pre_string, Identity_dyn_);

    // Note: if no interpolation, this part could be better formulated
    //---------------------------------------------------------------
    // Camera_measurement cost & constraints at T
    // Reserve cost for camera ahead of vo arrival
    // A_cam_pre_ * x_pre - A_cam_now_ * x_now - vcam_pre = b_cam_pre; for consequetive states

    b_cam_.segment(0, 3) << mhe_qp_.osqp_infinity * Vector3d::Ones(); // unconstrained place holder

    Q_cam_.setZero();
    Matrix3d Q_cam_dense = R_sb * Q_vo_p_ * R_sb.transpose();
    EigenUtils::SparseMatrixBlockAsignFromDense(Q_cam_, 0, 0, Q_cam_dense);

    mhe_qp_.addConstraints(Cam_Meas_string, -b_cam_, b_cam_);
    mhe_qp_.addConstraintDependency(Cam_Meas_string, X_T_pre_string, A_cam_pre_);
    mhe_qp_.addConstraintDependency(Cam_Meas_string, X_T_string, -A_cam_now_);
    mhe_qp_.addConstraintDependency(Cam_Meas_string, Vcam_T_pre_string, -Identity_cam_meas_);

    // 0.5 * || vcam_pre ||^2 _{Q_cam_pre}
    mhe_qp_.addCost(Cam_Meas_string, VectorXd::Zero(dim_cam_), Q_cam_);
    mhe_qp_.addCostDependency(Cam_Meas_string, Vcam_T_pre_string, Identity_cam_meas_);

    // Measurement cost at T
    //---------------------------------------------------------------
    GetMeasurement(T);
    R_sb = R_input_rotation_stack_.back();

    // // Q_meas_:
    // //  [   R_sb * J * cov_encoder_position^{-1} * J' * R_sb' ];
    Q_meas_.setZero();
    MatrixXd Q_meas_dense = MatrixXd::Zero(dim_meas_, dim_meas_);

    for (int i = 0; i < num_legs_; i++)
    {
        b_meas_.segment<3>(i * 3) = -R_sb * J_imu_2_foot_stack_.back().block<3, 3>(i * 3, 0) * joint_velocity_stack_.back().segment<3>(i * 3) -
                                   R_sb * angular_b_input_stack_.back().cross(p_imu_2_foot_stack_.back().block<3, 1>(i * 3, 0));
        if (contact_input_stack_.back()(i) == 0.0)
        {
            Q_meas_dense.block<3, 3>(i * 3, i * 3) << 1 / infinite_ * Matrix3d::Identity();
        }
        else
        {

            MatrixXd C_meas_dense = MatrixXd::Zero(3, 3);

            MatrixXd G_meas_i = MatrixXd::Zero(3, 9);

            G_meas_i.block<3, 3>(0, 0) = -J_imu_2_foot_stack_.back().block<3, 3>(i * 3, 0);

            Matrix3d omega_skew = Matrix3d::Zero();
            EigenUtils::vector3dSkew(omega_skew, angular_b_input_stack_.back());
            // std::cout << -omega_skew * J_imu_2_foot_stack_.back().block<3, 3>(i * 3, 0) << std::endl;
            G_meas_i.block<3, 3>(0, 3) = -omega_skew * J_imu_2_foot_stack_.back().block<3, 3>(i * 3, 0);

            Matrix3d kin_skew = Matrix3d::Zero();
            EigenUtils::vector3dSkew(kin_skew, p_imu_2_foot_stack_.back().block<3, 1>(i * 3, 0));
            G_meas_i.block<3, 3>(0, 6) = kin_skew;

            MatrixXd C_meas_i = MatrixXd::Zero(9, 9);
            C_meas_i.block<3, 3>(0, 0) = C_encoder_velocity_;
            C_meas_i.block<3, 3>(3, 3) = C_encoder_position_;
            C_meas_i.block<3, 3>(6, 6) = C_gyro_;

            C_meas_dense = R_sb * G_meas_i * C_meas_i *
                           G_meas_i.transpose() * R_sb.transpose();

            Q_meas_dense.block<3, 3>(i * 3, i * 3) = C_meas_dense.inverse();
        }
    }

    EigenUtils::SparseMatrixBlockAsignFromDense(Q_meas_, 0, 0, Q_meas_dense);

    // A_meas_ x_i - v_{i} = b_meas_
    mhe_qp_.addConstraints(Meas_string, b_meas_, b_meas_);
    mhe_qp_.addConstraintDependency(Meas_string, X_T_string, A_meas_);
    mhe_qp_.addConstraintDependency(Meas_string, V_T_string, -Identity_meas_);

    // 0.5 * ||v_{i} ||^2 _{Q_meas_}
    mhe_qp_.addCost(Meas_string, VectorXd::Zero(dim_meas_), Q_meas_);
    mhe_qp_.addCostDependency(Meas_string, V_T_string, Identity_meas_);

    mhe_qp_.updateQP(T);

    // ---------------------------------------------------------------
    // Debug log

    // std::string nameQ = "Q" + std::to_string(T);
    // Log2txt(Q_dyn_,nameQ);
    // std::string nameB = "Bdyn" + std::to_string(T);
    // Log2txt(Bdyn,nameB);
    // std::string nameA = "Adyn" + std::to_string(T);
    // Log2txt(Adyn,nameA);
    // std::string nameBm = "Bmeas" + std::to_string(T);
    // Log2txt(Bmeas,nameBm);
    // std::string nameQm = "Qm" + std::to_string(T);
    // Log2txt(Q_meas_,nameQm);
}

void DecentralizedEstimation::UpdateVOConstraints(int T)
{
    std::map<int, Vector3d> vo_constraints_idx_regs;
    std::map<int, double> vo_reliability_idx_regs;
    double vo_reliability = 1.0;

    for (int i = 0; i < vo_curve_.node_count() - 1; ++i)
    {
        Vector3d pose_world_idx = vo_curve_._distances[i + 1];

        int idx = (vo_insert_idx_stack_.back() + i) *
                      (dim_meas_ + dim_state_ + dim_cam_) +
                  dim_meas_ + dim_state_;
        vo_constraints_idx_regs.insert({idx, pose_world_idx});
        vo_reliability_idx_regs.insert({vo_insert_discrete_time_stack_.back() + i, vo_reliability});

        // update the constraintbound for the marginalization, no marginalization if unbounded
        std::string update_name = "VO_measurement_" + std::to_string(vo_insert_discrete_time_stack_.back() + i);
        mhe_qp_.updateConstraintBound(update_name, -pose_world_idx, -pose_world_idx, true);
        // mhe_qp_.updateCostGain(update_name, vo_reliability);
    }
    mhe_qp_.Update_Image_bound(vo_constraints_idx_regs, vo_reliability_idx_regs);
}

// Tested
void DecentralizedEstimation::GetMeasurement(int T)
{
    // get current measurement from robot_sub_ptr_
    R_sb_ = (robot_sub_ptr_->quaternion_.normalized()).toRotationMatrix();

    double imu_time = robot_sub_ptr_->imu_time_;
    Vector3d accel_b = robot_sub_ptr_->accel_b_; // returns the accelerometer readings in body frame;
    accel_s_ = R_sb_ * accel_b + gravity_;
    angular_b_ = robot_sub_ptr_->angular_b_;

    joint_position_ = robot_sub_ptr_->joint_states_position_;
    joint_velocity_ = robot_sub_ptr_->joint_states_velocity_;
    // joint_effort_ = robot_sub_ptr_->joint_states_effort_;

    p_imu_2_foot_ = MatrixXd::Zero(12, 1);
    J_imu_2_foot_ = MatrixXd::Zero(12, 3);

    // kinematics off_set from unitree go1 floating base to unitree imu
    // MatrixXd off_set = MatrixXd::Zero(1, 3);
    // off_set << -0.01592, -0.06659, -0.00617; // unitree base to unitree imu,     p_FL_foot = p_FL_foot - off_set;
    // MatrixXd off_set_ngimu_unitree_imu = MatrixXd::Zero(1, 3);
    // off_set_ngimu_unitree_imu << -0.0270, -0.0522, -0.1415; // ngimu to unitree imu

    // Foot order: FR, FL, RR, RL; For both hardware and kinematics lib
    // Joints Order: hip, thig, calf, foot(fixed 0); For both hardware and kinematics lib
    VectorXd joint_position_append = VectorXd::Zero(22); // [p,v, 4 foot * 4 joints, ]
    for (int i = 0; i < num_legs_; i++)
    {
        joint_position_append.segment<3>(6 + i * 4) = joint_position_.segment<3>(i * 3); // first 6 are dof of floating base
        joint_position_append(6 + i * 4 + 3) = 0.0;                                      // fixed foot joints
    }

    //------------------------------------------------------
    // FR
    MatrixXd p_FR_foot = MatrixXd::Zero(1, 3);
    SymFunction::FR_foot(p_FR_foot, joint_position_append);
    p_FR_foot = p_FR_foot + p_ib_;
    p_imu_2_foot_.block<3, 1>(0 * 3, 0) = p_FR_foot.transpose();

    MatrixXd J_FR_foot = MatrixXd::Zero(3, 22);
    SymFunction::J_FR(J_FR_foot, joint_position_append);
    J_imu_2_foot_.block<3, 3>(0 * 3, 0) = J_FR_foot.block<3, 3>(0, 6 + 0 * 4);

    // FL
    MatrixXd p_FL_foot = MatrixXd::Zero(1, 3);
    SymFunction::FL_foot(p_FL_foot, joint_position_append);
    p_FL_foot = p_FL_foot + p_ib_;
    p_imu_2_foot_.block<3, 1>(1 * 3, 0) = p_FL_foot.transpose();

    MatrixXd J_FL_foot = MatrixXd::Zero(3, 22);
    SymFunction::J_FL(J_FL_foot, joint_position_append);
    J_imu_2_foot_.block<3, 3>(1 * 3, 0) = J_FL_foot.block<3, 3>(0, 6 + 1 * 4);

    // RR
    MatrixXd p_RR_foot = MatrixXd::Zero(1, 3);
    SymFunction::RR_foot(p_RR_foot, joint_position_append);
    p_RR_foot = p_RR_foot + p_ib_;
    p_imu_2_foot_.block<3, 1>(2 * 3, 0) = p_RR_foot.transpose();

    MatrixXd J_RR_foot = MatrixXd::Zero(3, 22);
    SymFunction::J_RR(J_RR_foot, joint_position_append);
    J_imu_2_foot_.block<3, 3>(2 * 3, 0) = J_RR_foot.block<3, 3>(0, 6 + 2 * 4);

    // RL
    MatrixXd p_RL_foot = MatrixXd::Zero(1, 3);
    SymFunction::RL_foot(p_RL_foot, joint_position_append);
    p_RL_foot = p_RL_foot + p_ib_;
    p_imu_2_foot_.block<3, 1>(3 * 3, 0) = p_RL_foot.transpose();

    MatrixXd J_RL_foot = MatrixXd::Zero(3, 22);
    SymFunction::J_RL(J_RL_foot, joint_position_append);
    J_imu_2_foot_.block<3, 3>(3 * 3, 0) = J_RL_foot.block<3, 3>(0, 6 + 3 * 4);

    //------------------------------------------------------
    // contact
    contact_ = Vector4d::Zero();
    for (int i = 0; i < num_legs_; i++)
    {
        contact_(i) = (joint_position_(12 + i) >= contact_effort_theshold_) ? 1.0 : 0.0;
    }
    // std::cout << "contact_effort_: " << joint_position_.segment<4>(12) << std::endl;
    // std::cout << "contact_flag_: " << contact_ << std::endl;

    //------------------------------------------------------
    // Sychronize the VO frames to the nearest IMU frames to the left
    bool vo_new_meas_flag = robot_sub_ptr_->vo_new_;
    if (vo_new_meas_flag && imu_time_stack_.size() > 0) // log when new vo transformation was subscribed
    {

        vo_p_body_pre_2_body_ = robot_sub_ptr_->vo_p_body_pre_2_body_; // Relative translation between body frame in body_pre

        double vo_time_pre_ = robot_sub_ptr_->vo_time_pre_;
        double vo_time_now_ = robot_sub_ptr_->vo_time_now_;
        robot_sub_ptr_->vo_new_ = false; // meassage logged

        // sychronize VO timestamp to the first IMU timestamp to the left
        // IMU time is the rclcpp::clock.now() when the imu_callback gets callled in subscriber callback
        auto idx_pre_ptr = std::upper_bound(imu_time_stack_.begin(),
                                            imu_time_stack_.end(), vo_time_pre_); // find the first imu time bigger than vo_time_pre_

        if (idx_pre_ptr == imu_time_stack_.begin())
        {
            std::cout << "not storing enough imu info, failed to interpolate" << std::endl;
            // could happen at the beginning of the MHE
            // discard the vo_meas
            // this if condition should only happens at during the initalization, so give some time for the imu to store enough info
        }
        else
        {
            int imu_sychron_idx_pre = std::distance(imu_time_stack_.begin(), idx_pre_ptr) - 1;
            // double imu_sychron_time_pre = imu_time_stack_[imu_sychron_idx_pre];
            R_vo_sb_pre_ = R_input_rotation_stack_[imu_sychron_idx_pre]; // R_world_2_vo_pre, used to get relative translation between vo frames in world coordinates

            auto idx_now_ptr = std::upper_bound(imu_time_stack_.begin(),
                                                imu_time_stack_.end(), vo_time_now_); // find the first imu time bigger than vo_time_
            int imu_sychron_idx_now = std::distance(imu_time_stack_.begin(), idx_now_ptr) - 1;

            p_vo_accmulate_ += R_vo_sb_pre_ * vo_p_body_pre_2_body_; // accumulate translation in world frame, (exist a constant offset)

            int imu_window_start_idx = int(imu_time_stack_.size() - std::min(N_, T));
            int imu_interpolation_start_idx = std::max(imu_window_start_idx, imu_sychron_idx_pre); // interpolation starts from either the start of the window or image_frame_sychron_pre
            double time_interpolate_start = imu_time_stack_[imu_interpolation_start_idx];
            int discrete_time_interpolate_start = discrete_time_stack[imu_interpolation_start_idx]; // the discrete time used to find the appropriate VO_measurement_discrete_time

            // assuming the control points are uniform in time, the assumption is reasonable when using RealSense D455 + ORBSLAM3
            vo_curve_.add_way_point(p_vo_accmulate_, vo_time_now_); // the control point is consists of P_now, P_pre, P_{pre-1}, P_{pre-2}

            if (imu_sychron_idx_now > imu_window_start_idx && vo_curve_._way_points.size() >= 4)
            {
                double insert_relative_idx = imu_interpolation_start_idx - imu_window_start_idx; // the realtive idx in the window/horizon that the imterpolation starts
                double interpolate_num = imu_sychron_idx_now - imu_interpolation_start_idx + 1;  // interpolate using a constant time difference 1/N, also approximation

                vo_curve_.set_interval(time_interpolate_start, interpolate_num, dt_);

                // need to decide the knot vector here, otherwise the control point is treated as equaly spaced in time
                vo_curve_.interpolate_waypoint();

                vo_insert_idx_stack_.push_back(insert_relative_idx);
                vo_insert_discrete_time_stack_.push_back(discrete_time_interpolate_start);

                vo_to_be_processed_flag_ = true;
            }
            // vo_pose_body_pre_2_body_stack.push_back(p_vo_accmulate_);
            // R_vo_sb_rotation_stack.push_back(R_vo_sb_pre_); // record the R_world_2_body
            // vo_sychron_time_stack_.push_back(imu_sychron_time_pre);
            // vo_time_stack_.push_back(vo_time_pre_);
        }
    }

    //------------------------------------------------------
    // Stack and un-Stack
    imu_time_stack_.push_back(imu_time);
    discrete_time_stack.push_back(T);
    R_input_rotation_stack_.push_back(R_sb_);
    accel_s_input_stack_.push_back(accel_s_);
    p_imu_2_foot_stack_.push_back(p_imu_2_foot_);
    J_imu_2_foot_stack_.push_back(J_imu_2_foot_);
    contact_input_stack_.push_back(contact_);
    joint_velocity_stack_.push_back(joint_velocity_);
    angular_b_input_stack_.push_back(angular_b_);

    // Only keep the previous N_ measurements, but the storage size is arbitary since no stack.front is used
    // discrete time now: T;
    // discrete time optimization window start: T-N_
    // if (int(accel_s_input_stack_.size()) > N_ + 1) // should be N_+1 to store the measments in the current optimization window
    if (int(accel_s_input_stack_.size()) > 4 * N_ + 1)

    {
        discrete_time_stack.erase(discrete_time_stack.begin());
        R_input_rotation_stack_.erase(R_input_rotation_stack_.begin());
        accel_s_input_stack_.erase(accel_s_input_stack_.begin());
        p_imu_2_foot_stack_.erase(p_imu_2_foot_stack_.begin());
        J_imu_2_foot_stack_.erase(J_imu_2_foot_stack_.begin());
        contact_input_stack_.erase(contact_input_stack_.begin());

        imu_time_stack_.erase(imu_time_stack_.begin());
        joint_velocity_stack_.erase(joint_velocity_stack_.begin());
        angular_b_input_stack_.erase(angular_b_input_stack_.begin());
    }

    if (int(vo_insert_idx_stack_.size()) >= N_ + 1)
    {
        // R_vo_sb_rotation_stack_.erase(R_vo_sb_rotation_stack.begin());
        // vo_sychron_time_stack_.erase(vo_sychron_time_stack_.begin());
        // vo_time_stack_.erase(vo_time_stack_.begin());
        vo_insert_idx_stack_.erase(vo_insert_idx_stack_.begin());
        vo_insert_discrete_time_stack_.erase(vo_insert_discrete_time_stack_.begin());
    }
}

void DecentralizedEstimation::MarginalizeKF()
{
}

// KF part validated
void DecentralizedEstimation::InitializeKF()
{
    GetMeasurement(0);

    Matrix3d R_sb = R_input_rotation_stack_.back();

    for (int i = 0; i < num_legs_; i++)
    {
        b_meas_.segment<3>(i * 3) = R_sb * p_imu_2_foot_stack_.back().block<3, 1>(i * 3, 0);
    }

    // x_prior:
    // [    0                       ;   ]   p_s
    // [    0                       ;   ]   v_s
    // [    R_sb * leg_imu_2_foot_b ;   ]   p_foot_s
    // [    0                       ;   ]   accel_bias_b
    x_prior_.segment<3>(0) << 0.0, 0.0, 0.0; // best to start on the ground,
    x_prior_.segment<3>(3) << 0.0, 0.0, 0.0;
    x_prior_.segment(6, dim_meas_) = b_meas_;
    x_prior_.segment<3>(6 + dim_meas_) << 0.0, 0.0, 0.0;

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
    C_prior.block<3, 3>(12, 12) = std::pow(params_ptr_->foot_init_std_, 2) * MatrixXd::Identity(3, 3);
    C_prior.block<3, 3>(15, 15) = std::pow(params_ptr_->foot_init_std_, 2) * MatrixXd::Identity(3, 3);
    C_prior.block<3, 3>(18, 18) = std::pow(params_ptr_->accel_bias_init_std_, 2) * Matrix3d::Identity();

    C_KF_ = C_prior;
}

void DecentralizedEstimation::UpdateKF()
{
    GetMeasurement(0);

    Matrix3d R_sb = R_input_rotation_stack_.back();
    // C_meas
    //  [   R_sb * C_kinematic  * R_sb';    ]
    MatrixXd C_meas = MatrixXd::Zero(dim_meas_, dim_meas_);

    for (int i = 0; i < num_legs_; i++)
    {
        b_meas_.segment<3>(i * 3) = R_sb * p_imu_2_foot_stack_.back().block<3, 1>(i * 3, 0);
        // C_meas.block<3, 3>(i * 3, i * 3) = R_sb * J_imu_2_foot_stack_.back().block<3, 3>(i * 3, 0) *
        //                                    C_encoder_position_ *
        //                                    J_imu_2_foot_stack_.back().block<3, 3>(i * 3, 0).transpose() * R_sb.transpose();
        C_meas.block<3, 3>(i * 3, i * 3) = R_sb *
                                           C_encoder_position_ *
                                           R_sb.transpose();
    }
    // std::cout << "b_meas_:" << b_meas_ << std::endl;

    K_KF_ = C_KF_ * A_meas_.transpose() * (A_meas_ * C_KF_ * A_meas_.transpose() + C_meas).inverse();
    x_KF_ = x_KF_ + K_KF_ * (b_meas_ - A_meas_ * x_KF_);
    C_KF_ = (MatrixXd::Identity(dim_state_, dim_state_) - K_KF_ * A_meas_) * C_KF_;

    // std::cout << "correct" << K_KF_ * (b_meas_ - A_meas_ * x_KF_) << std::endl;
    // KF prediction update
    //---------------------------------------------------------------
    // Matrix3d R_sb = R_input_rotation_stack_.back();
    Vector3d accel_s = accel_s_input_stack_.back();
    double dt = dt_;
    std::cout << "dt" << dt << std::endl;
    // b_dyn_:
    // [    - 0.5 * dt^2 * accel_s; ]   p_s
    // [    - dt * accel_s        ; ]   v_s
    // [    0                     ; ]   p_foot_s
    // [    0                     ; ]   accel_bias_b
    b_dyn_.segment(0, 3) << -0.5 * dt * dt * accel_s;
    b_dyn_.segment(3, 3) << -dt * accel_s;

    // A_dyn_:
    // p_{i+1} = p_i + dt * v_i + 0.5 * dt^2 * accel_s - 0.5 * dt^2 * R_sb * accel_b_bias + w_p;
    // v_{i+1} = v_i + dt * accel_s - dt * R_sb * accel_b_bias + w_v;
    // p_foot_{i+1} = p_foot_{i} + w_foot
    // accel_b_bias_{i+1} = accel_b_bias_{i} + w_a_bias
    // [    I   dt* I   0   - 0.5 * dt^2 * R_sb;  ]
    // [    0   I       0   - dt * R_sb        ;  ]
    // [    0   0       I   0                  ;  ]
    // [    0   0       0   I                  ;  ]
    A_dyn_.setIdentity();

    EigenUtils::SparseMatrixBlockAsignFromDense(A_dyn_, 0, 3, dt * Matrix3d::Identity());
    EigenUtils::SparseMatrixBlockAsignFromDense(A_dyn_, 0, 18, -dt * dt / 2 * R_sb);
    EigenUtils::SparseMatrixBlockAsignFromDense(A_dyn_, 3, 18, -dt * R_sb);
    x_KF_ = A_dyn_ * x_KF_ - b_dyn_;

    double infinite_ = params_ptr_->foot_swing_std_;

    MatrixXd C_dyn = MatrixXd::Zero(dim_state_, dim_state_);

    // Uncertainty from input
    // ---------------------------------------------------------------
    // G_dyn:
    // [  R_sb * dt          -0.5 * R_sb *dt^2   0           0;      ]
    // [  0                  -R_sb *dt           0           0;      ]
    // [  0                  0                   R_sb * dt   0;      ]
    // [  0                  0                   0           I * dt; ]
    // C_dyn = G_dyn * diag[C_velocity, C_accel_, C_slip, C_accel_bias_  ] * G_dyn'
    // C_slip = infinite_, if contact(feet) = false

    MatrixXd G_dyn = MatrixXd::Zero(dim_state_, dim_state_);
    G_dyn.block<3, 3>(0, 0) = R_sb * dt;
    G_dyn.block<3, 3>(0, 3) = -0.5 * R_sb * dt * dt;
    G_dyn.block<3, 3>(3, 3) = -R_sb * dt;
    G_dyn.block<3, 3>(6, 6) = R_sb * dt;
    G_dyn.block<3, 3>(9, 9) = R_sb * dt;
    G_dyn.block<3, 3>(12, 12) = R_sb * dt;
    G_dyn.block<3, 3>(15, 15) = R_sb * dt;
    G_dyn.block<3, 3>(18, 18) = Matrix3d::Identity() * dt;

    MatrixXd C_input = MatrixXd::Zero(dim_state_, dim_state_);
    C_input.block<3, 3>(0, 0) = C_p_;
    C_input.block<3, 3>(3, 3) = C_accel_;
    C_input.block<3, 3>(6, 6) = C_foot_slide_;
    C_input.block<3, 3>(9, 9) = C_foot_slide_;
    C_input.block<3, 3>(12, 12) = C_foot_slide_;
    C_input.block<3, 3>(15, 15) = C_foot_slide_;
    C_input.block<3, 3>(18, 18) = C_accel_bias_;
    for (int i = 0; i < num_legs_; i++)
    {
        // left foot covariance
        if (contact_input_stack_.back()(i) == 0.0) // if contact at foot_idx, fixed the stance foot position using large Q_foot_slide
        {
            C_input.block<3, 3>(6 + 3 * i, 6 + 3 * i) << infinite_ * Matrix3d::Identity();
        }
    }

    C_dyn = G_dyn * C_input * G_dyn.transpose();
    // std::cout << C_dyn << std::endl;
    C_KF_ = A_dyn_ * C_KF_ * A_dyn_.transpose() + C_dyn;
    // std::cout << "x_KF"<< x_KF_<< std::endl;
    //---------------------------------------------------------------
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
