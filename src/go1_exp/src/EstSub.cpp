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

        joint_state_sub = create_subscription<sensor_msgs::msg::JointState>("/unitree/joint_state",
                                                                            10,
                                                                            std::bind(&robotSub::joint_state_callback, this, std::placeholders::_1));

        // Sparsly integrated VO
        vo_sub = create_subscription<orbslam3_msgs::msg::VoRealtiveTransform>("orb/vo",
                                                                              10,
                                                                              std::bind(&robotSub::vo_callback, this, std::placeholders::_1));
        vo_pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>("orb/pos",
                                                                           10,
                                                                           std::bind(&robotSub::vo_pose_callback, this, std::placeholders::_1));

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

    void robotSub::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        robot_store_->joint_states_position_ = Map<VectorXd, Unaligned>(const_cast<double *>(msg->position.data()), msg->position.size());
        robot_store_->joint_states_velocity_ = Map<VectorXd, Unaligned>(const_cast<double *>(msg->velocity.data()), msg->velocity.size());
        // robot_store_->joint_states_effort_ = Map<VectorXd, Unaligned>(const_cast<double *>(msg->effort.data()), msg->effort.size());
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

        QuaternionToEuler(mocap_quaternion_, gt_euler_);

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

        QuaternionToEuler(robot_store_->quaternion_, filter_euler_);
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

    void robotSub::vo_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

    void robotSub::QuaternionToEuler(Quaterniond quaternion, Vector3d &euler)
    {
        double roll, pitch, yaw = 0.0;
        double qw = quaternion.w();
        double qx = quaternion.x();
        double qy = quaternion.y();
        double qz = quaternion.z();
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (qw * qx + qy * qz);
        double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        roll = atan2(sinr_cosp, cosr_cosp);
        // pitch (y-axis rotation)
        double sinp = 2 * (qw * qy - qz * qx);
        if (abs(sinp) >= 1)
            pitch = copysign(M_PI / 2, sinp); // Use M_PI for pi in C++
        else
            pitch = asin(sinp);
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        yaw = atan2(siny_cosp, cosy_cosp);
        euler(0) = roll;
        euler(1) = pitch;
        euler(2) = yaw;
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
        this->declare_parameter("foot_init_std", 0.025);
        this->declare_parameter("accel_bias_init_std", 0.1);

        robot_params_->p_init_std_ = this->get_parameter("p_init_std").as_double_array();
        robot_params_->v_init_std_ = this->get_parameter("v_init_std").as_double_array();
        robot_params_->foot_init_std_ = this->get_parameter("foot_init_std").as_double();
        robot_params_->accel_bias_init_std_ = this->get_parameter("accel_bias_init_std").as_double();

        this->declare_parameter("joint_position_std", std::vector<double>{0.01, 0.01, 0.01});
        this->declare_parameter("joint_velocity_std", std::vector<double>{0.01, 0.01, 0.01});
        this->declare_parameter("foot_slide_std", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("foot_swing_std", 1000000000.0);
        this->declare_parameter("contact_effort_theshold", 150.0);
        this->declare_parameter("p_ib", std::vector<double>{0.0, 0.0, 0.0});

        robot_params_->joint_position_std_ = this->get_parameter("joint_position_std").as_double_array();
        robot_params_->joint_velocity_std_ = this->get_parameter("joint_velocity_std").as_double_array();
        robot_params_->foothold_slide_std_ = this->get_parameter("foot_slide_std").as_double_array();
        robot_params_->foot_swing_std_ = this->get_parameter("foot_swing_std").as_double();
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
