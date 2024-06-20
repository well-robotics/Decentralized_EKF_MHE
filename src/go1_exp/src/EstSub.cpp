#include "EstSub.hpp"

using namespace Eigen;

namespace robotSub
{

    robotSub::robotSub(const std::string &name) : Node(name)
    {
        robot_store_ = std::make_shared<robot_store>();
        robot_params_ = std::make_shared<robot_params>();

        paramsWrapper();
        est_type_ = robot_params_->est_type_;
        int timer_interval_ = this->get_parameter("interval").as_int();

        timer_ = create_wall_timer(std::chrono::milliseconds(timer_interval_), std::bind(&robotSub::timerCallback, this));

        // Go1 sensors
        imu_sub = create_subscription<sensor_msgs::msg::Imu>("/unitree/imu",
                                                             10,
                                                             std::bind(&robotSub::imu_callback, this, std::placeholders::_1));

        lo_sub = create_subscription<sensor_msgs::msg::JointState>("/unitree/joint_state",
                                                                   10,
                                                                   std::bind(&robotSub::lo_callback, this, std::placeholders::_1));

        // Sparsly integrated VO
        vo_sub = create_subscription<orbslam3_msgs::msg::VoRealtiveTransform>("orb/vo",
                                                                              10,
                                                                              std::bind(&robotSub::vo_callback, this, std::placeholders::_1));
        vo_p_sub = create_subscription<geometry_msgs::msg::PoseStamped>("orb/pos",
                                                                        10,
                                                                        std::bind(&robotSub::vo_p_callback, this, std::placeholders::_1));

        // Decentralized Orientation
        orien_filter_sub = create_subscription<sensor_msgs::msg::Imu>("imu/filter",
                                                                      10,
                                                                      std::bind(&robotSub::orien_filter_callback, this, std::placeholders::_1));

        // Ground Truth
        mocap_sub = create_subscription<optitrack_broadcast::msg::Mocap>("/mocap/RigidBody",
                                                                         10,
                                                                         std::bind(&robotSub::mocap_callback, this, std::placeholders::_1));

        time_0_ = static_cast<double>(rclcpp::Clock().now().nanoseconds()) / 1e9;
    }

    robotSub::~robotSub()
    {
    }

    void robotSub::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {

        rclcpp::Time imu_time = rclcpp::Clock().now();
        robot_store_->imu_time_ = static_cast<double>(imu_time.nanoseconds()) / 1e9 - time_0_; // Correctly obtaining the timestamp in seconds as a double

        robot_store_->accel_b_(0) = msg->linear_acceleration.x;
        robot_store_->accel_b_(1) = msg->linear_acceleration.y;
        robot_store_->accel_b_(2) = msg->linear_acceleration.z;
        
        robot_store_->angular_b_(0) = msg->angular_velocity.x;
        robot_store_->angular_b_(1) = msg->angular_velocity.y;
        robot_store_->angular_b_(2) = msg->angular_velocity.z;

        msg_num++;
    }

    void robotSub::lo_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {

        robot_store_->joint_states_position_ = Map<VectorXd, Unaligned>(const_cast<double *>(msg->position.data()), msg->position.size());
        robot_store_->joint_states_velocity_ = Map<VectorXd, Unaligned>(const_cast<double *>(msg->velocity.data()), msg->velocity.size());
        // robot_store_->joint_states_effort_ = Map<VectorXd, Unaligned>(const_cast<double *>(msg->effort.data()), msg->effort.size());

        VectorXd joint_position_append = VectorXd::Zero(22); // [p,v, 4 foot * 4 joints, ]
        VectorXd contact = VectorXd::Zero(robot_params_->num_legs_);
        for (int i = 0; i < robot_params_->num_legs_; i++)
        {
            joint_position_append.segment<3>(6 + i * 4) = robot_store_->joint_states_position_.segment<3>(i * 3); // first 6 are dof of floating base
            joint_position_append(6 + i * 4 + 3) = 0.0;                                                           // fixed foot joints
            contact(i) = (robot_store_->joint_states_position_(12 + i) >= robot_params_->contact_effort_theshold_) ? 1.0 : 0.0;
        }

        MatrixXd p_imu_2_foot_ = MatrixXd::Zero(12, 1);
        MatrixXd J_imu_2_foot_ = MatrixXd::Zero(12, 3);
        MatrixXd p_ib = MatrixXd::Zero(1, 3);
        p_ib << robot_params_->p_ib_[0], robot_params_->p_ib_[1], robot_params_->p_ib_[2];
        // Foot order: FR, FL, RR, RL; For both hardware and kinematics lib
        // Joints Order: hip, thig, calf, foot(fixed 0); For both hardware and kinematics lib
        MatrixXd p_FR_foot = MatrixXd::Zero(1, 3);
        SymFunction::FR_foot(p_FR_foot, joint_position_append);
        p_FR_foot = p_FR_foot + p_ib;
        p_imu_2_foot_.block<3, 1>(0 * 3, 0) = p_FR_foot.transpose();

        MatrixXd J_FR_foot = MatrixXd::Zero(3, 22);
        SymFunction::J_FR(J_FR_foot, joint_position_append);
        J_imu_2_foot_.block<3, 3>(0 * 3, 0) = J_FR_foot.block<3, 3>(0, 6 + 0 * 4);

        // FL
        MatrixXd p_FL_foot = MatrixXd::Zero(1, 3);
        SymFunction::FL_foot(p_FL_foot, joint_position_append);
        p_FL_foot = p_FL_foot + p_ib;
        p_imu_2_foot_.block<3, 1>(1 * 3, 0) = p_FL_foot.transpose();

        MatrixXd J_FL_foot = MatrixXd::Zero(3, 22);
        SymFunction::J_FL(J_FL_foot, joint_position_append);
        J_imu_2_foot_.block<3, 3>(1 * 3, 0) = J_FL_foot.block<3, 3>(0, 6 + 1 * 4);

        // RR
        MatrixXd p_RR_foot = MatrixXd::Zero(1, 3);
        SymFunction::RR_foot(p_RR_foot, joint_position_append);
        p_RR_foot = p_RR_foot + p_ib;
        p_imu_2_foot_.block<3, 1>(2 * 3, 0) = p_RR_foot.transpose();

        MatrixXd J_RR_foot = MatrixXd::Zero(3, 22);
        SymFunction::J_RR(J_RR_foot, joint_position_append);
        J_imu_2_foot_.block<3, 3>(2 * 3, 0) = J_RR_foot.block<3, 3>(0, 6 + 2 * 4);

        // RL
        MatrixXd p_RL_foot = MatrixXd::Zero(1, 3);
        SymFunction::RL_foot(p_RL_foot, joint_position_append);
        p_RL_foot = p_RL_foot + p_ib;
        p_imu_2_foot_.block<3, 1>(3 * 3, 0) = p_RL_foot.transpose();

        MatrixXd J_RL_foot = MatrixXd::Zero(3, 22);
        SymFunction::J_RL(J_RL_foot, joint_position_append);
        J_imu_2_foot_.block<3, 3>(3 * 3, 0) = J_RL_foot.block<3, 3>(0, 6 + 3 * 4);

        robot_store_->p_imu_2_foot_ = p_imu_2_foot_;
        robot_store_->J_imu_2_foot_ = J_imu_2_foot_;
        robot_store_->contact_ = contact;
    }

    void robotSub::mocap_callback(const optitrack_broadcast::msg::Mocap::SharedPtr msg)
    {

        robot_store_->gt_p_(0) = msg->position[0];
        robot_store_->gt_p_(1) = msg->position[1];
        robot_store_->gt_p_(2) = msg->position[2];

        robot_store_->gt_v_s_(0) = msg->velocity[0];
        robot_store_->gt_v_s_(1) = msg->velocity[1];
        robot_store_->gt_v_s_(2) = msg->velocity[2];

        // Test with GT orientation from Mocap
        // robot_store_->quaternion_.x() = msg->quaternion[1];
        // robot_store_->quaternion_.y() = msg->quaternion[2];
        // robot_store_->quaternion_.z() = msg->quaternion[3];
        // robot_store_->quaternion_.w() = msg->quaternion[0];

        // Quaterniond mocap_quaternion;
        mocap_quaternion_.x() = msg->quaternion[1];
        mocap_quaternion_.y() = msg->quaternion[2];
        mocap_quaternion_.z() = msg->quaternion[3];
        mocap_quaternion_.w() = msg->quaternion[0];

        EigenUtils::QuaternionToEuler(mocap_quaternion_, gt_euler_);

        init_mocap_ = true;

        gt_p_ = robot_store_->gt_p_ - gt_p_offset_;
        gt_v_b_ = mocap_quaternion_.normalized().toRotationMatrix() * robot_store_->gt_v_s_;
    }

    void robotSub::orien_filter_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {

        robot_store_->quaternion_.x() = msg->orientation.x;
        robot_store_->quaternion_.y() = msg->orientation.y;
        robot_store_->quaternion_.z() = msg->orientation.z;
        robot_store_->quaternion_.w() = msg->orientation.w;

        EigenUtils::QuaternionToEuler(robot_store_->quaternion_, filter_euler_);
    }

    void robotSub::vo_callback(const orbslam3_msgs::msg::VoRealtiveTransform::SharedPtr msg)
    {
        // sub for the vo_pose_body_pre_2_body,
        robot_store_->vo_new_ = true;
        robot_store_->vo_time_pre_ = static_cast<double>(msg->header_pre.stamp.sec) +
                                     static_cast<double>(msg->header_pre.stamp.nanosec) / 1e9 - time_0_; // image_pre time stamp
        robot_store_->vo_time_now_ = static_cast<double>(msg->header.stamp.sec) +
                                     static_cast<double>(msg->header.stamp.nanosec) / 1e9 - time_0_; // image_pre time stamp
        robot_store_->vo_p_body_pre_2_body_(0) = msg->x_relative;
        robot_store_->vo_p_body_pre_2_body_(1) = msg->y_relative;
        robot_store_->vo_p_body_pre_2_body_(2) = msg->z_relative;
    }

    void robotSub::vo_p_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        vo_p_(0) = msg->pose.position.x;
        vo_p_(1) = msg->pose.position.y;
        vo_p_(2) = msg->pose.position.z;

        vo_quaternion_.x() = msg->pose.orientation.x;
        vo_quaternion_.y() = msg->pose.orientation.y;
        vo_quaternion_.z() = msg->pose.orientation.z;
        vo_quaternion_.w() = msg->pose.orientation.w;

        vo_p_ = vo_quaternion_offset_ * vo_p_;

        if (init_mocap_ && !init_vo_)
        {
            vo_quaternion_offset_ = mocap_quaternion_ * vo_quaternion_.inverse(); // R_IMU_correct =  (R_Mocap * R_IMU^T) * R_IMU
            init_vo_ = true;
        }
    }

    void robotSub::timerCallback()
    {
        auto start_callback = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count();

        if (msg_num >= 10)
        {

            if (discrete_time == 0)
            {
                mhe.initialize(robot_store_, robot_params_);
                gt_p_offset_ = robot_store_->gt_p_;
            }
            else
            {
                mhe.update(discrete_time);
            }
            discrete_time++;

            // logging
            if (discrete_time == robot_params_->N_ + 1)
            {
                init_logging();
            }
            if (discrete_time > robot_params_->N_ + 1)
            {

                logger.spin_logging();
            }
        }

        auto stop_callback = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count();
        auto duration_callback = static_cast<double>(stop_callback - start_callback) / 1000000;
        std::cout << 1 / duration_callback << std::endl;
    }

    void robotSub::init_logging()
    {
        if (robot_params_->est_type_ == 0)
        {
            std::string log_name = "mhe_" + this->get_parameter("log_name").as_string();

            logger.init(log_name);
            logger.add_data(gt_p_, "pose");
            logger.add_data(gt_v_b_, "GT_v");
            logger.add_data(mhe.v_MHE_b_, "v_body");
            logger.add_data(mhe.x_MHE_, "x_MHE");
            logger.add_data(mhe.p_vo_accmulate_, "p_vo_accmulate_");
            logger.add_data(filter_euler_, "filter_euler_");
            logger.add_data(gt_euler_, "gt_euler_");
        }
        else
        {
            std::string log_name = "kf_" + this->get_parameter("log_name").as_string();

            logger.init(log_name);
            logger.add_data(gt_p_, "pose");
            logger.add_data(gt_v_b_, "GT_v");
            logger.add_data(mhe.v_KF_b_, "v_body");
            logger.add_data(mhe.x_KF_, "x_MHE");
            logger.add_data(mhe.p_vo_accmulate_, "p_vo_accmulate_");
            logger.add_data(filter_euler_, "filter_euler_");
            logger.add_data(gt_euler_, "gt_euler_");
        }
    }

    void robotSub::paramsWrapper()
    {
        this->declare_parameter<std::string>("log_name", "exp");

        this->declare_parameter("p_process_std", std::vector<double>{0.01, 0.01, 0.01});
        this->declare_parameter("accel_input_std", std::vector<double>{0.01, 0.04, 0.001});
        this->declare_parameter("gyro_input_std", std::vector<double>{0.01, 0.01, 0.01});
        this->declare_parameter("accel_bias_process_std", std::vector<double>{1., 1., 0.1});

        robot_params_->p_process_std_ = this->get_parameter("p_process_std").as_double_array();
        robot_params_->accel_input_std_ = this->get_parameter("accel_input_std").as_double_array();
        robot_params_->gyro_input_std_ = this->get_parameter("gyro_input_std").as_double_array();
        robot_params_->accel_bias_std_ = this->get_parameter("accel_bias_process_std").as_double_array();

        this->declare_parameter("p_init_std", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("v_init_std", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("foot_init_std", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("accel_bias_init_std", std::vector<double>{0.001, 0.001, 0.001});

        robot_params_->p_init_std_ = this->get_parameter("p_init_std").as_double_array();
        robot_params_->v_init_std_ = this->get_parameter("v_init_std").as_double_array();
        robot_params_->foot_init_std_ = this->get_parameter("foot_init_std").as_double_array();
        robot_params_->accel_bias_init_std_ = this->get_parameter("accel_bias_init_std").as_double_array();

        this->declare_parameter("num_leg", 4);
        this->declare_parameter("leg_odom_type", 0);
        this->declare_parameter("joint_position_std", std::vector<double>{0.01, 0.01, 0.01});
        this->declare_parameter("joint_velocity_std", std::vector<double>{0.01, 0.01, 0.01});
        this->declare_parameter("foot_slide_std", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("foot_swing_std", std::vector<double>{10000.0, 10000.0, 10000.0});
        this->declare_parameter("contact_effort_theshold", 150.0);
        this->declare_parameter("p_ib", std::vector<double>{0.0, 0.0, 0.0});

        robot_params_->num_legs_ = this->get_parameter("num_leg").as_int();
        robot_params_->leg_odom_type_ = this->get_parameter("leg_odom_type").as_int();
        robot_params_->joint_position_std_ = this->get_parameter("joint_position_std").as_double_array();
        robot_params_->joint_velocity_std_ = this->get_parameter("joint_velocity_std").as_double_array();
        robot_params_->foot_slide_std_ = this->get_parameter("foot_slide_std").as_double_array();
        robot_params_->foot_swing_std_ = this->get_parameter("foot_swing_std").as_double_array();
        robot_params_->contact_effort_theshold_ = this->get_parameter("contact_effort_theshold").as_double();
        robot_params_->p_ib_ = this->get_parameter("p_ib").as_double_array();

        this->declare_parameter("vo_p_std", std::vector<double>{0.001, 0.001, 0.001});
        robot_params_->vo_p_std_ = this->get_parameter("vo_p_std").as_double_array();

        this->declare_parameter("rate", 50);
        this->declare_parameter("interval", 20);
        this->declare_parameter("N", 50);
        this->declare_parameter("est_type", 0);

        robot_params_->rate_ = this->get_parameter("rate").as_int();
        robot_params_->N_ = this->get_parameter("N").as_int();
        robot_params_->est_type_ = this->get_parameter("est_type").as_int();

        this->declare_parameter("rho", 0.1);
        this->declare_parameter("alpha", 1.6);
        this->declare_parameter("delta", 0.00001);
        this->declare_parameter("sigma", 0.00001);
        this->declare_parameter("verbose", true);
        this->declare_parameter("adaptRho", true);
        this->declare_parameter("polish", true);
        this->declare_parameter("maxQPIter", 1000);
        this->declare_parameter("primTol", 0.000001);
        this->declare_parameter("dualTol", 0.000001);
        this->declare_parameter("realtiveTol", 1e-3);
        this->declare_parameter("absTol", 1e-3);
        this->declare_parameter("timeLimit", 0.005);

        // osqp parameters
        robot_params_->rho_ = this->get_parameter("rho").as_double();
        robot_params_->alpha_ = this->get_parameter("alpha").as_double();
        robot_params_->delta_ = this->get_parameter("delta").as_double();
        robot_params_->sigma_ = this->get_parameter("sigma").as_double();
        robot_params_->verbose_ = this->get_parameter("verbose").as_bool();
        robot_params_->adaptRho_ = this->get_parameter("adaptRho").as_bool();
        robot_params_->polish_ = this->get_parameter("polish").as_bool();
        robot_params_->maxQPIter_ = this->get_parameter("maxQPIter").as_int();
        robot_params_->primTol_ = this->get_parameter("primTol").as_double();
        robot_params_->dualTol_ = this->get_parameter("dualTol").as_double();
        robot_params_->realtiveTol_ = this->get_parameter("realtiveTol").as_double();
        robot_params_->absTol_ = this->get_parameter("absTol").as_double();
        robot_params_->timeLimit_ = this->get_parameter("timeLimit").as_double();
    }
}
