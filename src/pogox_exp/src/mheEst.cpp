#include "../include/mheEst.hpp"
#include <fstream>

//  comments with math
//  debugging break using the ptr with the matrix

//  eigen operater = vs <<
// eigen block
//  add ros msg typr to eigen lib
//  ros2 params shorter
//  wrapper

MheQuadrupedalEstimation::MheQuadrupedalEstimation()
{
}

// Set the Estimation parameters and QP solver
void MheQuadrupedalEstimation::initialize(std::shared_ptr<robot_store> sub, std::shared_ptr<mhe_params> params)
{
    robot_sub_ptr_ = sub;
    params_ptr_ = params;
    est_type_ = params_ptr_->est_type_;

    dt = 1.0 / params_ptr_->rate_;
    N = params_ptr_->N_;

    mhe_qp.setHorizon(N, dim_state, dim_meas, dim_cam);

    gravity_ << 0, 0, -9.81;

    //---------------------------------------------------------------
    // Gain setup from params; Q: gains, inv of covariance; C: covariance
    infinite = params_ptr_->foot_swing_std_;

    StdVec2CovMat(params_ptr_->p_process_std_, C_p);

    StdVec2CovMat(params_ptr_->accel_input_std_, C_accel);
    StdVec2CovMat(params_ptr_->accel_input_contact_std_, C_accel_contact);
    StdVec2CovMat(params_ptr_->accel_input_impact_std_, C_accel_impact);

    StdVec2GainMat(params_ptr_->accel_bias_std_, Q_accel_bias);

    StdVec2GainMat(params_ptr_->foothold_slide_std_, Q_foot_slide);

    StdVec2GainMat(params_ptr_->vo_pose_std_, Q_vo_pose);

    StdVec2CovMat(params_ptr_->lidar_std_, C_lidar);

    // osqp setup from params
    //---------------------------------------------------------------
    mhe_qp.osqp.settings()->setWarmStart(true);
    mhe_qp.osqp.settings()->setAdaptiveRho(params_ptr_->adaptRho_);
    mhe_qp.osqp.settings()->setVerbosity(params_ptr_->verbose_);
    mhe_qp.osqp.settings()->setPolish(params_ptr_->polish_);
    mhe_qp.osqp.settings()->setMaxIteration(params_ptr_->maxQPIter_);
    mhe_qp.osqp.settings()->setRho(params_ptr_->rho_);
    mhe_qp.osqp.settings()->setAlpha(params_ptr_->alpha_);
    mhe_qp.osqp.settings()->setDelta(params_ptr_->delta_);
    mhe_qp.osqp.settings()->setSigma(params_ptr_->sigma_);
    mhe_qp.osqp.settings()->setRelativeTolerance(params_ptr_->realtiveTol_);
    mhe_qp.osqp.settings()->setAbsoluteTolerance(params_ptr_->absTol_);
    mhe_qp.osqp.settings()->setPrimalInfeasibilityTollerance(params_ptr_->primTol_);
    mhe_qp.osqp.settings()->setDualInfeasibilityTollerance(params_ptr_->dualTol_);
    mhe_qp.osqp.settings()->setTimeLimit(params_ptr_->timeLimit_);

    // Declair the sparse Identity matrix
    Identity_dyn.resize(dim_state, dim_state);
    Identity_dyn.setIdentity();

    Identity_cam_meas.resize(dim_cam, dim_cam);
    Identity_cam_meas.setIdentity();

    Identity_meas.resize(dim_meas, dim_meas);
    Identity_meas.setIdentity();

    Q_prior.resize(dim_state, dim_state);
    Q_prior.setZero();

    A_dyn.resize(dim_state, dim_state);
    A_dyn.setIdentity();

    Q_dyn.resize(dim_state, dim_state);
    Q_dyn.setZero();

    A_meas.resize(dim_meas, dim_state);
    A_meas.setZero();
    Q_meas.resize(dim_meas, dim_meas);
    Q_meas.setZero();

    A_cam_pre.resize(dim_cam, dim_state);
    A_cam_pre.setZero();

    A_cam_now.resize(dim_cam, dim_state);
    A_cam_now.setZero();

    Q_cam.resize(dim_cam, dim_cam);
    Q_cam.setZero();

    // Const Matrix setup
    //---------------------------------------------------------------
    // A_meas, *x:
    //   [  -I  0   I   0;  ]
    //   [  [0;0;1] 0 0 0;  ]
    MatrixXd A_meas_dense = MatrixXd::Zero(dim_meas, dim_state);
    A_meas_dense.block<3, 3>(0, 0) << -Matrix3d::Identity();
    A_meas_dense.block<3, 3>(0, 6) << Matrix3d::Identity();
    A_meas_dense(3, 2) = 1.0;
    eigenUtils::SparseMatrixBlockAsignFromDense(A_meas, 0, 0, A_meas_dense);

    // A_cam_pre_/A_cam_now_, *x:
    //   diag[  I   0   0   0;  ]
    A_cam_pre.coeffRef(0, 0) = 1.0;
    A_cam_pre.coeffRef(1, 1) = 1.0;
    A_cam_pre.coeffRef(2, 2) = 1.0;
    A_cam_now.coeffRef(0, 0) = 1.0;
    A_cam_now.coeffRef(1, 1) = 1.0;
    A_cam_now.coeffRef(2, 2) = 1.0;

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

void MheQuadrupedalEstimation::update(int T)
{

    switch (est_type_)
    {
    case 0:
    {

        UpdateMHE(T);
        // std::cout << "updateMHE to " + std::to_string(T) << std::endl;

        if (vo_to_be_processed_flag)
        {
            AssembleImage(T);
            vo_to_be_processed_flag = false;
        }

        if (T >= N)
        {
            mhe_qp.marginalizeQP(T - N); // the marginalization is independent of update() and assemble_image()
            // std::cout << "marginalizeMHE to " + std::to_string(T - N) << std::endl;
        }
        // tic("init");
        mhe_qp.initQP(T); // update the osqp, note to keep the sparsity structure
        // toc("init");
        // std::cout << "finish init " + std::to_string(T) << std::endl;

        // tic("solve");
        mhe_qp.solveQP();
        // toc("solve");
        // std::cout << "finish solve " + std::to_string(T) << std::endl;

        std::string var_name = "x_" + std::to_string(T);
        mhe_qp.getsolution(T);

        x_MHE = mhe_qp.x_solution_now;
        x_MHE_traj = mhe_qp.solution;
        // std::cout << "finish compute " + std::to_string(T) << std::endl;

        break;
    }
    case 1:
    {
        UpdateKF();
        break;
    }
    }
}

void MheQuadrupedalEstimation::InitializeMHE()
{

    GetMeasurement(0);

    Matrix3d R_sb = R_input_rotation_stack.back();

    b_meas.segment<3>(0) << R_sb * y_measure_imu_2_foot_stack.back();
    b_meas(3) = -lidar_measure_imu_2_foot_stack.back()(2);

    // Q_meas:
    //  [   R_sb * C_lidar^{-1} * R_sb' 0                           ;
    //      0                           (R_sb * C_lidar^{-1} * R_sb')(2,2) ;  ]
    Matrix3d Q_meas_dense = R_sb *
                            C_lidar.inverse() *
                            R_sb.transpose();

    MatrixXd Q_meas_dense_z(1, 1);
    Q_meas_dense_z(0, 0) = 1 / params_ptr_->lidar_z_std_ / params_ptr_->lidar_z_std_;
    Q_meas.setZero();

    eigenUtils::SparseMatrixBlockAsignFromDense(Q_meas, 0, 0, Q_meas_dense);
    eigenUtils::SparseMatrixBlockAsignFromDense(Q_meas, 3, 3, Q_meas_dense_z);

    // x_prior:
    // [    0; 0; 0.56              ;   ]   p_s
    // [    0                       ;   ]   v_s
    // [    R_sb * leg_imu_2_foot_b ;   ]   p_foot_s
    // [    0                       ;   ]   accel_bias_b
    x_prior.segment<3>(0) << 0.0, 0.0, b_meas(0); // best to start on the ground, but with lidar, the z could be self correcting
    x_prior.segment<3>(3) << 0.0, 0.0, 0.0;
    x_prior.segment<3>(6) << 0.0, 0.0, 0.0;
    // x_prior.segment<3>(9) << 0.02, -0.04, 0.08;
    x_prior.segment<3>(9) << 0.0, 0.0, 0.0;

    Matrix3d Q_pint = Matrix3d::Zero();
    Matrix3d Q_vint = Matrix3d::Zero();

    StdVec2GainMat(params_ptr_->p_init_std_, Q_pint);
    StdVec2GainMat(params_ptr_->v_init_std_, Q_vint);

    // Q_prior_0:
    //   [  Q_p_init  0          0             0                ;   ]
    //   [  0         Q_v_init   0             0                ;   ]
    //   [  0         0          Q_foot_init   0                ;   ]
    //   [  0         0          0             Q_accel_bias_init;   ]
    Q_prior.setZero();
    eigenUtils::SparseMatrixBlockAsignFromDense(Q_prior, 0, 0, Q_pint);
    eigenUtils::SparseMatrixBlockAsignFromDense(Q_prior, 3, 3, Q_vint);
    eigenUtils::SparseMatrixBlockAsignFromDense(Q_prior, 6, 6, 1 / std::pow(params_ptr_->foot_init_std_, 2) * Matrix3d::Identity());
    eigenUtils::SparseMatrixBlockAsignFromDense(Q_prior, 9, 9, 1 / std::pow(params_ptr_->accel_bias_init_std_, 2) * Matrix3d::Identity());

    // std::cout << "x0----------------------------" << std::endl;
    // std::cout << x_prior << std::endl;

    // || x_0 - x_prior||^2_{Q_prior}
    mhe_qp.addVariable("x_0", dim_state);
    mhe_qp.addCost("Prior_0", x_prior, Q_prior);
    mhe_qp.addCostDependency("Prior_0", "x_0", Identity_dyn);

    // A_meas x_i - v_{i} = b_meas
    mhe_qp.addVariable("v_0", dim_meas);
    mhe_qp.addConstraints("Measurement_0", b_meas, b_meas);
    mhe_qp.addConstraintDependency("Measurement_0", "x_0", A_meas);
    mhe_qp.addConstraintDependency("Measurement_0", "v_0", -Identity_meas);

    // || v_i ||^2_{Q_meas}
    mhe_qp.addCost("Measurement_0", VectorXd::Zero(dim_meas), Q_meas);
    mhe_qp.addCostDependency("Measurement_0", "v_0", Identity_meas);

    mhe_qp.updateQP(0);
}

void MheQuadrupedalEstimation::UpdateMHE(int T)
{
    //  |x_T - f( x_{T-1},u_{T-1} )|Q_{T-1} as |Adyn X_{T-1} - x_T - b_dyn|Q_{T-1}

    // Dynamic cost at T-1
    //---------------------------------------------------------------
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

    mhe_qp.addVariable(W_T_pre_string, dim_state);
    mhe_qp.addVariable(Vcam_T_pre_string, dim_cam);
    mhe_qp.addVariable(X_T_string, dim_state);
    mhe_qp.addVariable(V_T_string, dim_meas);

    Matrix3d R_sb = R_input_rotation_stack.back();
    Vector3d accel_s = accel_s_input_stack.back();

    // b_dyn:
    // [    - 0.5 * dt^2 * accel_s; ]   p_s
    // [    - dt * accel_s        ; ]   v_s
    // [    0                     ; ]   p_foot_s
    // [    0                     ; ]   accel_bias_b
    b_dyn.segment(0, 3) << -dt * dt / 2 * accel_s;
    b_dyn.segment(3, 3) << -dt * accel_s;

    A_dyn.setIdentity();
    // A_dyn:
    // [    I   dt* I   0   - 0.5 * dt^2 * R_sb;  ]
    // [    0   I       0   - dt * R_sb        ;  ]
    // [    0   0       I   0                  ;  ]
    // [    0   0       0   I                  ;  ]
    eigenUtils::SparseMatrixBlockAsignFromDense(A_dyn, 0, 3, dt * Matrix3d::Identity());
    eigenUtils::SparseMatrixBlockAsignFromDense(A_dyn, 0, 9, -dt * dt / 2 * R_sb);
    eigenUtils::SparseMatrixBlockAsignFromDense(A_dyn, 3, 9, -dt * R_sb);

    // Uncertainty from input
    // ---------------------------------------------------------------
    // G_dyn:
    // [  R_sb * dt          -0.5 * R_sb *dt^2   0           0;      ]
    // [  0                  -R_sb *dt           0           0;      ]
    // [  0                  0                   R_sb * dt   0;      ]
    // [  0                  0                   0           I * dt; ]
    // (G_dyn * diag[C_velocity, C_accel, C_slip, C_accel_bias  ] * G_dyn').inverse()
    // C_slip = infinite, if contact(feet) = false

    MatrixXd G_dyn_pv = MatrixXd::Zero(6, 6);
    G_dyn_pv.block<3, 3>(0, 0) = R_sb * dt;
    G_dyn_pv.block<3, 3>(0, 3) = 0.5 * R_sb * dt * dt;
    G_dyn_pv.block<3, 3>(3, 3) = R_sb * dt;

    MatrixXd C_dyn_pv = MatrixXd::Zero(6, 6);
    C_dyn_pv.block<3, 3>(0, 0) = C_p;
    C_dyn_pv.block<3, 3>(3, 3) = C_accel;
    if (contact_input_stack.back())
    {
        if (Contact_sum_ < 10)
        {
            //     // b_dyn = b_dyn / 200;
            // b_dyn = b_dyn * 0;

            C_dyn_pv.block<3, 3>(3, 3) = C_accel_impact;
        }
        else
        {
            C_dyn_pv.block<3, 3>(3, 3) = C_accel_contact;
        }
    }
    MatrixXd Q_dyn_pv = (G_dyn_pv * C_dyn_pv * G_dyn_pv.transpose()).inverse();

    Q_dyn.setZero();

    eigenUtils::SparseMatrixBlockAsignFromDense(Q_dyn, 0, 0, Q_dyn_pv);

    if (contact_input_stack.back()) // if contact at foot_idx, fixed the stance foot position using large Q_foot_slide
    {
        eigenUtils::SparseMatrixBlockAsignFromDense(Q_dyn,
                                                    6, 6,
                                                    1 / (dt * dt) *
                                                        R_sb * Q_foot_slide * R_sb.transpose());
    }
    else // if no contact at foot_idx, putting small gains on stance foot position
    {
        eigenUtils::SparseMatrixBlockAsignFromDense(Q_dyn,
                                                    6, 6,
                                                    1 / (dt * dt) / infinite * Matrix3d::Identity());
    }

    eigenUtils::SparseMatrixBlockAsignFromDense(Q_dyn,
                                                9, 9,
                                                1 / (dt * dt) * Q_accel_bias);

    // A_dyn x_i - x_{i+1} - w_{i} = b_dyn
    mhe_qp.addConstraints(Dyn_string, b_dyn, b_dyn);
    mhe_qp.addConstraintDependency(Dyn_string, W_T_pre_string, -Identity_dyn);
    mhe_qp.addConstraintDependency(Dyn_string, X_T_string, -Identity_dyn);
    mhe_qp.addConstraintDependency(Dyn_string, X_T_pre_string, A_dyn);

    // 0.5 * ||w_{i} ||^2 _{Q_dyn}
    mhe_qp.addCost(Dyn_string, VectorXd::Zero(12), Q_dyn);
    mhe_qp.addCostDependency(Dyn_string, W_T_pre_string, Identity_dyn);

    // Camera_measurement cost at T
    //---------------------------------------------------------------
    // Reserve cost for camera ahead of vo arrival
    // A_cam_pre * x_pre - A_cam_now * x_now - vcam_pre = b_cam_pre
    // After interpolation:
    // A_cam_pre * x_pre - A_cam_now * x_now - vcam_pre = b_cam_pre

    b_cam.segment(0, 3) << mhe_qp.osqp_infinity * Vector3d::Ones();

    Matrix3d Q_cam_dense = R_sb * Q_vo_pose * R_sb.transpose();
    if (contact_input_stack.back())
    {
        Q_cam_dense = 1.0 * Q_cam_dense;
    }
    Q_cam.setZero();
    eigenUtils::SparseMatrixBlockAsignFromDense(Q_cam, 0, 0, Q_cam_dense);

    mhe_qp.addConstraints(Cam_Meas_string, -b_cam, b_cam);
    mhe_qp.addConstraintDependency(Cam_Meas_string, X_T_pre_string, A_cam_pre);
    mhe_qp.addConstraintDependency(Cam_Meas_string, X_T_string, -A_cam_now);

    mhe_qp.addConstraintDependency(Cam_Meas_string, Vcam_T_pre_string, -Identity_cam_meas);

    // 0.5 * ||vcam_pre ||^2 _{Q_cam_pre}
    mhe_qp.addCost(Cam_Meas_string, VectorXd::Zero(dim_cam), Q_cam);
    mhe_qp.addCostDependency(Cam_Meas_string, Vcam_T_pre_string, Identity_cam_meas);

    // tic("Meas");

    GetMeasurement(T);

    // Measurement cost at T
    //---------------------------------------------------------------
    R_sb = R_input_rotation_stack.back();

    // b_meas:
    b_meas.segment<3>(0) << R_sb * y_measure_imu_2_foot_stack.back();
    b_meas(3) = -lidar_measure_imu_2_foot_stack.back()(2);

    // Q_meas:
    //  [   R_sb * C_lidar^{-1} * R_sb' ;   ]
    // Matrix3d Q_meas_dense = R_sb *
    //                         C_lidar.inverse() *
    //                         R_sb.transpose();
    Matrix3d Q_meas_dense = R_sb *
                            C_lidar.inverse() *
                            R_sb.transpose();
    MatrixXd Q_meas_dense_z(1, 1);
    Q_meas_dense_z(0, 0) = 1 / params_ptr_->lidar_z_std_ / params_ptr_->lidar_z_std_;

    Q_meas.setZero();
    eigenUtils::SparseMatrixBlockAsignFromDense(Q_meas, 0, 0, Q_meas_dense);
    eigenUtils::SparseMatrixBlockAsignFromDense(Q_meas, 3, 3, Q_meas_dense_z);
    // A_meas x_i - v_{i} = b_meas
    mhe_qp.addConstraints(Meas_string, b_meas, b_meas);
    mhe_qp.addConstraintDependency(Meas_string, X_T_string, A_meas);
    mhe_qp.addConstraintDependency(Meas_string, V_T_string, -Identity_meas);

    // 0.5 * ||v_{i} ||^2 _{Q_meas}
    mhe_qp.addCost(Meas_string, VectorXd::Zero(dim_meas), Q_meas);
    mhe_qp.addCostDependency(Meas_string, V_T_string, Identity_meas);

    mhe_qp.updateQP(T);
    // toc("update_qp");

    // data log
    // ---------------------------------------------------------------
    // std::string nameQ = "Q" + std::to_string(T);
    // Log2txt(Q_dyn,nameQ);
    // std::string nameB = "Bdyn" + std::to_string(T);
    // Log2txt(Bdyn,nameB);
    // std::string nameA = "Adyn" + std::to_string(T);
    // Log2txt(Adyn,nameA);
    // std::string nameBm = "Bmeas" + std::to_string(T);
    // Log2txt(Bmeas,nameBm);
    // std::string nameQm = "Qm" + std::to_string(T);
    // Log2txt(Q_meas,nameQm);
}

void MheQuadrupedalEstimation::AssembleImage(int T)
{
    std::map<int, Vector3d> vo_constraints_idx_regs;
    std::map<int, double> vo_reliability_idx_regs;
    double vo_reliability = 1.0;
    // if (vo_matches_stack.back() < 100)
    // {
    //     // vo_reliability = 0.0001;
    //     vo_reliability = 1.0;
    // }

    for (int i = 0; i < vo_curve.node_count() - 1; ++i)
    {

        Vector3d pose_world_1_2 = vo_curve._distances[i + 1];
        // if (vo_matches_stack.back() == 1)
        // {
        //     // vo_reliability = 0.0001;
        //     pose_world_1_2 = 0.01 * vo_curve._distances[i + 1];
        // }

        int idx = (vo_insert_idx_stack.back() + i) *
                      (dim_meas + dim_state + dim_cam) +
                  dim_meas + dim_state;
        vo_constraints_idx_regs.insert({idx, pose_world_1_2});
        vo_reliability_idx_regs.insert({vo_insert_discrete_time_stack.back() + i, vo_reliability});

        // update the constraintbound for the marginalization, no marginalization if unbounded
        std::string update_name = "VO_measurement_" + std::to_string(vo_insert_discrete_time_stack.back() + i);
        mhe_qp.updateConstraintBound(update_name, -pose_world_1_2, -pose_world_1_2, true);
        mhe_qp.updateCostGain(update_name, vo_reliability);
    }
    mhe_qp.Update_Image_bound(vo_constraints_idx_regs, vo_reliability_idx_regs);
}

// Tested
void MheQuadrupedalEstimation::GetMeasurement(int T)
{
    // get current measurement from sub_ptr
    // ngimu filer returns R_body_world
    // offset to align with Mocap orientation, to compare with GroundTruth
    // R_sb_ = robot_sub_ptr_->offset_quaternion_.normalized().toRotationMatrix() *
    //         robot_sub_ptr_->quaternion_.normalized().toRotationMatrix().transpose();
    // R_sb_ = robot_sub_ptr_->quaternion_.normalized().toRotationMatrix().transpose();
    R_sb_ = robot_sub_ptr_->quaternion_.normalized().toRotationMatrix();

    double imu_time = robot_sub_ptr_->imu_time_;
    Vector3d accel_b = robot_sub_ptr_->accel_b_; // returns the gravity free acceleration in body frame;
    accel_s_ = R_sb_ * R_ib_ * accel_b + gravity_;
    // accel_s_ = R_sb_ * R_ib_ * accel_b;
    // std::cout << accel_s_ << std::endl;
    Contact_ = robot_sub_ptr_->stance_;
    if (Contact_)
    {
        Contact_sum_ += Contact_;
    }
    else
    {
        Contact_sum_ = 0;
    }

    VectorXd lidar_b = VectorXd::Zero(3);

    lidar_b(2) = -robot_sub_ptr_->distance_ - 0.26; // lidar target position realtive to imu in body frame

    Vector3d lidar_s_ = R_sb_ * lidar_b;

    VectorXd p_foot_ = Vector3d::Zero(3);

    if (Contact_ == 0.0)
    {
        p_foot_(2) = -0.3 - 0.26;
    }
    else
    {
        p_foot_ = lidar_b;
    }

    bool vo_new_inf = robot_sub_ptr_->vo_new_;
    if (vo_new_inf && imu_time_stack.size() > 0) // log when new vo transformation was subscribed
    {

        if (std::abs(vo_pose_body_pre_2_body_.norm() - robot_sub_ptr_->vo_pose_body_pre_2_body_.norm()) < 0.025)
        {
            vo_pose_body_pre_2_body_ = robot_sub_ptr_->vo_pose_body_pre_2_body_; // Relative translation between body frame in body_pre
        }
        else
        {
            vo_pose_body_pre_2_body_ = 0.95 * vo_pose_body_pre_2_body_ + 0.05 * robot_sub_ptr_->vo_pose_body_pre_2_body_;
        }
        // vo_pose_body_pre_2_body_ = robot_sub_ptr_->vo_pose_body_pre_2_body_; // Relative translation between body frame in body_pre

        double vo_time_now_ = robot_sub_ptr_->vo_time_now_;
        double vo_time_pre_ = robot_sub_ptr_->vo_time_pre_;
        robot_sub_ptr_->vo_new_ = false; // meassage logged
        // sychronize vo timestamp to the left first imu timestamp
        // imu time is the rclcpp::clock.now() when the imu_callback gets callled
        auto idx_pre_ptr = std::upper_bound(imu_time_stack.begin(),
                                            imu_time_stack.end(), vo_time_pre_); // find the first imu time bigger than vo_time_pre_

        if (idx_pre_ptr == imu_time_stack.begin())
        {
            std::cout << "not storing enough imu info, failed to have accurate interpolation" << std::endl;
            // should happens at the beginning of the MHE
            // discard the vo_meas
        }
        else // this if condition should only happens at during the initalization, so give some time for the imu to store enough info
        {
            int imu_sychron_pre_idx = std::distance(imu_time_stack.begin(), idx_pre_ptr) - 1;

            double t_imu_sychron_pre = imu_time_stack[imu_sychron_pre_idx];
            R_vo_sb_pre_ = R_input_rotation_stack[imu_sychron_pre_idx]; // R_world_2_vo_pre, used to get relative translation between vo frames in world coordinates

            auto idx_now_ptr = std::upper_bound(imu_time_stack.begin(),
                                                imu_time_stack.end(), vo_time_now_); // find the first imu time bigger than vo_time_
            int imu_sychron_now_idx = std::distance(imu_time_stack.begin(), idx_now_ptr) - 1;

            x_body_accmulate += R_vo_sb_pre_ * vo_pose_body_pre_2_body_; // accumulate translation in world frame, (exist a constant offset)

            int t_imu_window_start_idx = int(imu_time_stack.size() - std::min(N, T));
            int imu_interpolation_start_idx = std::max(t_imu_window_start_idx, imu_sychron_pre_idx); // interpolation starts from either the start of the window or image_sychron_pre
            double t_interpolate_start = imu_time_stack[imu_interpolation_start_idx];
            int discrete_time_interpolate_start = discrete_time_stack[imu_interpolation_start_idx]; // the discrete time used to find the appropriate VO_measurement_discrete_time

            // assuming the control points are uniform in time, even it's not
            vo_curve.add_way_point(x_body_accmulate, vo_time_now_); // the control point is consists of P_now, P_pre, P_{pre-1}, P_{pre-2}

            if (imu_sychron_now_idx > t_imu_window_start_idx && vo_curve._way_points.size() >= 4)
            {
                double insert_relative_idx = imu_interpolation_start_idx - t_imu_window_start_idx; // the realtive discrete time that the imterpolation starts
                double interpolate_num = imu_sychron_now_idx - imu_interpolation_start_idx + 1;    // interpolate using a constant time difference 1/N, also approximation

                vo_curve.set_interval(t_interpolate_start, interpolate_num, dt);

                // need to decide the knot vector here, otherwise the control point is treated as equaly spaced in time
                vo_curve.interpolate_waypoint();

                vo_insert_idx_stack.push_back(insert_relative_idx);
                vo_insert_discrete_time_stack.push_back(discrete_time_interpolate_start);
                vo_to_be_processed_flag = true;
            }
            // vo_pose_body_pre_2_body_stack.push_back(x_body_accmulate);
            R_vo_sb_rotation_stack.push_back(R_vo_sb_pre_); // record the R_world_2_body
            vo_sychron_time_stack.push_back(t_imu_sychron_pre);
            vo_time_stack.push_back(vo_time_pre_);
            int vo_matches_ = Contact_;

            vo_matches_stack.push_back(vo_matches_);
            // if (0) // debug
            // {
            //     std::cout << "t_imu_sychron_pre" << t_imu_sychron_pre << std::endl;
            //     std::cout << "imu_sychron_pre_idx" << imu_sychron_pre_idx << std::endl;
            //     std::cout << "imu_sychron_now_idx" << imu_sychron_now_idx << std::endl;
            //     std::cout << "imu_sychron_now" << imu_time_stack[imu_sychron_now_idx] << std::endl;
            //     std::cout << "window start idx: " << int(imu_time_stack.size() - std::min(N, T)) << std::endl;
            //     std::cout << "imu_start_time:" << imu_time_stack[int(imu_time_stack.size() - std::min(N, T))] << std::endl;
            //     std::cout << "imu_end_time:" << imu_time << std::endl;
            //     std::cout << "image_lagging:" << imu_time - vo_time_now_ << std::endl;
            //     std::cout << "interpolate_num" << interpolate_num << std::endl;
            //     std::cout << "insert_relative_idx" << insert_relative_idx << std::endl;
            //     std::cout << "imu_interpolation_start_idx" << imu_interpolation_start_idx << std::endl;
            // }
        }
    }

    discrete_time_stack.push_back(T);
    imu_time_stack.push_back(imu_time);
    accel_s_input_stack.push_back(accel_s_);
    R_input_rotation_stack.push_back(R_sb_);
    contact_input_stack.push_back(Contact_);
    lidar_measure_imu_2_foot_stack.push_back(lidar_s_);
    y_measure_imu_2_foot_stack.push_back(p_foot_);

    // only keep the previous N measurements, but the storage size is arbitary since no stack.front is used
    // discrete time now: T;
    // discrete time optimization window start: T-N
    // discrete time storage start: T - 4*N;

    if (int(accel_s_input_stack.size()) > 4 * N + 1) // should be n+1 to store the measments in the current optimization window
    {
        discrete_time_stack.erase(discrete_time_stack.begin());

        imu_time_stack.erase(imu_time_stack.begin());
        accel_s_input_stack.erase(accel_s_input_stack.begin());
        R_input_rotation_stack.erase(R_input_rotation_stack.begin());
        contact_input_stack.erase(contact_input_stack.begin());
        lidar_measure_imu_2_foot_stack.erase(lidar_measure_imu_2_foot_stack.begin());
        y_measure_imu_2_foot_stack.erase(y_measure_imu_2_foot_stack.begin());
    }

    if (int(vo_insert_idx_stack.size()) >= N + 1)
    {
        R_vo_sb_rotation_stack.erase(R_vo_sb_rotation_stack.begin());
        vo_sychron_time_stack.erase(vo_sychron_time_stack.begin());
        vo_time_stack.erase(vo_time_stack.begin());
        vo_insert_idx_stack.erase(vo_insert_idx_stack.begin());
        vo_insert_discrete_time_stack.erase(vo_insert_discrete_time_stack.begin());
    }
}

// // Tested
// void MheQuadrupedalEstimation::GetMeasurement(int T)
// {
//     // get current measurement from sub_ptr
//     X_quaternion = robot_sub_ptr_->quaternion_.normalized();
//     R_sb_ = robot_sub_ptr_->quaternion_.normalized().toRotationMatrix().transpose();

//     v_sb_ = robot_sub_ptr_->v_sb_; // ground truth

//     double imu_time = robot_sub_ptr_->imu_time_;
//     Vector3d accel_b = robot_sub_ptr_->accel_b_;
//     // std::cout << "no gravity compensate: " << R_sb_ * R_ib_ * accel_b << std::endl;
//     accel_s_ = R_sb_ * R_ib_ * accel_b + gravity_;
//     // std::cout << "gravity compensate: " << accel_s_ << std::endl;

//     Contact_ = robot_sub_ptr_->stance_;
//     // std::cout << "contact: " << Contact_ << std::endl;

//     Vector3d lidar_b = Vector3d::Zero();
//     lidar_b.segment<1>(2) << -0.29 - robot_sub_ptr_->distance_; // foot position realtive to imu in body frame
//     Vector3d lidar_s_ = R_sb_ * lidar_b;

//     Vector3d p_foot_ = Vector3d::Zero();
//     if (Contact_ == 0.0)
//     {
//         p_foot_(2) = -0.57;
//     }
//     else
//     {
//         p_foot_(2) = lidar_b(2);
//     }
//     // VectorXd var = VectorXd::Zero(7);
//     // var[6] = -0.2;
//     // MatrixXd out = MatrixXd::Zero(1,3);
//     // SymFunction::pToe(out,var);
//     // std::cout << "contact: " << Contact_ << std::endl;
//     // std::cout << "out: " << out << std::endl;
//     // std::cout << "pfoot: " << p_foot_ << std::endl;
//     // std::cout << "lidar_s" << lidar_s_ << std::endl;

//     bool vo_new_inf = robot_sub_ptr_->vo_new_;
//     if (vo_new_inf && imu_time_stack.size() > 0) // log when new vo transformation was subscribed
//     {
//         vo_pose_body_pre_2_body_ = robot_sub_ptr_->vo_pose_body_pre_2_body_; // Realative translation between body frame in body_pre
//         double vo_time_now_ = robot_sub_ptr_->vo_time_now_;
//         double vo_time_pre_ = robot_sub_ptr_->vo_time_pre_;
//         robot_sub_ptr_->vo_new_ = false; // meassage logged

//         // sychronize vo timestamp to the left first imu timestamp
//         auto idx_pre_ptr = std::upper_bound(imu_time_stack.begin(),
//                                             imu_time_stack.end(), vo_time_pre_); // find the first imu time bigger than vo_time_pre_

//         if (idx_pre_ptr == imu_time_stack.begin())
//         {
//             std::cout << "not storing enough imu info, failed to have accurate interpolation" << std::endl;
//             // should check with the control points, but currently left with a large imu time storage
//             // discard the vo_meas
//         }
//         else // this if condition should only happens at during the initalization, so give some time for the imu to store enough info
//         {
//             int imu_sychron_pre_idx = std::distance(imu_time_stack.begin(), idx_pre_ptr) - 1;
//             double t_imu_sychron_pre = imu_time_stack[imu_sychron_pre_idx];
//             // std::cout << "t_imu_sychron_pre" << t_imu_sychron_pre << std::endl;

//             R_vo_sb_pre_ = R_input_rotation_stack[imu_sychron_pre_idx]; // interpolate to get the correct R_vo_sb_pre here, write approximate to test

//             auto idx_now_ptr = std::upper_bound(imu_time_stack.begin(),
//                                                 imu_time_stack.end(), vo_time_now_); // find the first imu time bigger than vo_time_pre_
//             int imu_sychron_now_idx = std::distance(imu_time_stack.begin(), idx_now_ptr) - 1;
//             x_body_accmulate += R_vo_sb_pre_ * vo_pose_body_pre_2_body_; // accumulate translation in world frame, (exist a constant offset)

//             // std::cout << "imu_sychron_pre_idx" << imu_sychron_pre_idx << std::endl;
//             // std::cout << "imu_sychron_now_idx" << imu_sychron_now_idx << std::endl;
//             // std::cout << "imu_sychron_now" << imu_time_stack[imu_sychron_now_idx] << std::endl;
//             // std::cout << "window start idx: " << int(imu_time_stack.size() - std::min(N, T)) << std::endl;
//             // std::cout << "imu_start_time:" << imu_time_stack[int(imu_time_stack.size() - std::min(N, T))] << std::endl;
//             // std::cout << "imu_end_time:" << imu_time << std::endl;
//             // std::cout << "image_lagging:" << imu_time - vo_time_now_ << std::endl;

//             // t_imu_window_start_idx = imu_time_stack.size()-N; the interpolation starts at max(window_start, imu_sychron_image_pre_)
//             int imu_interpolation_start_idx = std::max(int(imu_time_stack.size() - N), imu_sychron_pre_idx);
//             double t_interpolate_start = imu_time_stack[imu_interpolation_start_idx];
//             int discrete_time_interpolate_start = discrete_time_stack[imu_interpolation_start_idx];

//             vo_curve.add_way_point(x_body_accmulate, vo_time_now_); // the control point is consists of P_now, P_pre, P_{pre-1}, P_{pre-2}

//             if (imu_sychron_now_idx > int(imu_time_stack.size() - std::min(N, T)) && vo_curve._way_points.size() >= 4)
//             {
//                 // std::cout << "imu_interpolation_start_idx" << imu_interpolation_start_idx << std::endl;
//                 // std::cout << "insert_relative_idx:" << imu_interpolation_start_idx - int(imu_time_stack.size() - N) << std::endl;

//                 double insert_relative_idx = imu_interpolation_start_idx - int(imu_time_stack.size() - std::min(N, T)); // the realtive discrete time that the imterpolation starts
//                 double interpolate_num = imu_sychron_now_idx - imu_interpolation_start_idx + 1;                         // interpolate using a constant time difference 1/N, also approximation

//                 vo_curve.set_interval(t_interpolate_start, interpolate_num, dt);

//                 // std::cout << "interpolate_num" << interpolate_num << std::endl;
//                 // std::cout << "insert_relative_idx" << insert_relative_idx << std::endl;
//                 // std::cout << "imu_interpolation_start_idx" << imu_interpolation_start_idx << std::endl;
//                 // need to decide the knot vector here, otherwise the control point is treated as equaly spaced in time

//                 vo_curve.interpolate_waypoint();
//                 // std::cout << "nodes num: " << vo_curve._nodes.size() << std::endl;

//                 // for (int i = 1; i < interpolate_num; i++)
//                 // {
//                 //     x_body_interpolate_accmulate += vo_curve._distances[i];
//                 // }
//                 vo_insert_idx_stack.push_back(insert_relative_idx);
//                 vo_insert_discrete_time_stack.push_back(discrete_time_interpolate_start);
//                 vo_to_be_processed_flag = true;
//             }
//             // vo_pose_body_pre_2_body_stack.push_back(x_body_accmulate);
//             R_vo_sb_rotation_stack.push_back(R_vo_sb_pre_); // record the R_world_2_body
//             vo_sychron_time_stack.push_back(t_imu_sychron_pre);
//             vo_time_stack.push_back(vo_time_pre_);
//         }
//     }

//     discrete_time_stack.push_back(T);

//     imu_time_stack.push_back(imu_time);
//     accel_s_input_stack.push_back(accel_s_);
//     R_input_rotation_stack.push_back(R_sb_);

//     y_measure_imu_2_foot_stack.push_back(p_foot_);
//     lidar_measure_imu_2_foot_stack.push_back(lidar_s_);

//     contact_input_stack.push_back(Contact_);

//     // only keep the previous N measurements
//     // discrete time now: T;
//     // discrete time optimization window start: T-N
//     // discrete time storage start: T - 2*N;

//     if (int(accel_s_input_stack.size()) > 4 * N + 1) // should be n+1 to store the measments in the current optimization window
//     {
//         discrete_time_stack.erase(discrete_time_stack.begin());

//         imu_time_stack.erase(imu_time_stack.begin());
//         accel_s_input_stack.erase(accel_s_input_stack.begin());
//         R_input_rotation_stack.erase(R_input_rotation_stack.begin());
//         y_measure_imu_2_foot_stack.erase(y_measure_imu_2_foot_stack.begin());
//         lidar_measure_imu_2_foot_stack.erase(lidar_measure_imu_2_foot_stack.begin());
//         contact_input_stack.erase(contact_input_stack.begin());
//     }

//     if (int(vo_insert_idx_stack.size()) >= N + 1)
//     {
//         R_vo_sb_rotation_stack.erase(R_vo_sb_rotation_stack.begin());
//         vo_sychron_time_stack.erase(vo_sychron_time_stack.begin());
//         vo_time_stack.erase(vo_time_stack.begin());
//         vo_insert_idx_stack.erase(vo_insert_idx_stack.begin());
//         vo_insert_discrete_time_stack.erase(vo_insert_discrete_time_stack.begin());
//     }
// }

void MheQuadrupedalEstimation::StdVec2CovMat(const std::vector<double> &std, Matrix3d &Cov)
{
    Cov.diagonal() << std::pow(std[0], 2),
        std::pow(std[1], 2),
        std::pow(std[2], 2);
}

void MheQuadrupedalEstimation::StdVec2GainMat(const std::vector<double> &std, Matrix3d &Gain)
{
    Gain.diagonal() << 1 / std::pow(std[0], 2),
        1 / std::pow(std[1], 2),
        1 / std::pow(std[2], 2);
}

void MheQuadrupedalEstimation::Log2txt(const MatrixXd matrix, std::string filename)
{
    if (log)
    {
        IOFormat CleanFmt(10, 0, ", ", "\n", "[", "]");
        std::string path = "/home/jkang/mhe_ros/logtxt/";
        std::ofstream outfile(path + filename + ".txt");
        outfile << matrix.format(CleanFmt);
        outfile.close();
    }
}

void MheQuadrupedalEstimation::tic(std::string str, int mode)
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

void MheQuadrupedalEstimation::toc(std::string str) { tic(str, 1); }

void MheQuadrupedalEstimation::MarginalizeKF()
{
}

// KF part validated
void MheQuadrupedalEstimation::InitializeKF()
{
}

void MheQuadrupedalEstimation::UpdateKF()
{
}

void MheQuadrupedalEstimation::reset()
{

    mhe_qp.resetQP();
}