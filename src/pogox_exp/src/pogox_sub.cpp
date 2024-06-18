#include "pogox_sub.hpp"

using namespace Eigen;

namespace pogoxSub
{

    pogoxSub::pogoxSub(const std::string &name) : Node(name)
    {
        pogox_store = std::make_shared<robot_store>();
        robot_params_ = std::make_shared<mhe_params>();

        paramsWrapper();

        est_type_ = robot_params_->est_type_;

        int timer_interval_ = this->get_parameter("interval").as_int();

        timer_ = create_wall_timer(std::chrono::milliseconds(timer_interval_), std::bind(&pogoxSub::timerCallback, this));

        // ros1
        ngimu_sub = create_subscription<sensor_msgs::msg::Imu>("ngimu/imu",
                                                               10,
                                                               std::bind(&pogoxSub::imu_callback, this, std::placeholders::_1));
        orientation_sub = create_subscription<geometry_msgs::msg::Quaternion>("ngimu/quaternion",
                                                                              10,
                                                                              std::bind(&pogoxSub::orientation_callback, this, std::placeholders::_1));
        contact_sub = create_subscription<optitrack_broadcast::msg::SensorFloat>("contact",
                                                                                 10,
                                                                                 std::bind(&pogoxSub::contact_callback, this, std::placeholders::_1));
        lidar_sub = create_subscription<optitrack_broadcast::msg::SensorFloat>("distance",
                                                                               10,
                                                                               std::bind(&pogoxSub::lidar_callback, this, std::placeholders::_1));
        mocap_sub = create_subscription<optitrack_broadcast::msg::Mocap>("mocap/RigidBody",
                                                                         10,
                                                                         std::bind(&pogoxSub::mocap_callback, this, std::placeholders::_1));
        // vins_sub = create_subscription<nav_msgs::msg::Odometry>("/vins_fusion/imu_propagate",
        //                                                         10,
        //                                                         std::bind(&pogoxSub::vins_callback, this, std::placeholders::_1));

        // ros
        vo_sub = create_subscription<orbslam3_msgs::msg::VoRealtiveTransform>("orb/vo",
                                                                              10,
                                                                              std::bind(&pogoxSub::vo_callback, this, std::placeholders::_1));

        vo_pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>("orb/pos",
                                                                           10,
                                                                           std::bind(&pogoxSub::vo_pose_callback, this, std::placeholders::_1));

        imu_filter_sub = create_subscription<sensor_msgs::msg::Imu>("imu/filter",
                                                                    10,
                                                                    std::bind(&pogoxSub::imu_filter_callback, this, std::placeholders::_1));
        // ros2
        footimu_sub = create_subscription<communication::msg::FootImu>("/hardware/ngimu_sensor_ACM1",
                                                                       10,
                                                                       std::bind(&pogoxSub::foot_imu_callback, this, std::placeholders::_1));
        socket_sub = create_subscription<custom_msgs::msg::SensorFloat>("/sensor_data",
                                                                        10,
                                                                        std::bind(&pogoxSub::socket_callback, this, std::placeholders::_1));
        vrpn_sub = create_subscription<geometry_msgs::msg::PoseStamped>("/vrpn_mocap/RigidBody/pose",
                                                                        10,
                                                                        std::bind(&pogoxSub::vrpn_callback, this, std::placeholders::_1));
        vrpn_vel_sub = create_subscription<geometry_msgs::msg::Twist>("/vrpn/vel",
                                                                      10,
                                                                      std::bind(&pogoxSub::vrpn_vel_callback, this, std::placeholders::_1));

        openvins_sub = create_subscription<nav_msgs::msg::Odometry>("/ov_msckf/odomimu",
                                                                    10,
                                                                    std::bind(&pogoxSub::openvins_callback, this, std::placeholders::_1));

        publisher_EST_base_ = create_publisher<geometry_msgs::msg::Pose>("ESTbase_c", 10);
        publisher_GT_base_ = create_publisher<geometry_msgs::msg::Pose>("GTbase_c", 10);

        publisher_velocity_GT_ = create_publisher<geometry_msgs::msg::Twist>("GT_v_c", 10);
        publisher_velocity_EST_ = create_publisher<geometry_msgs::msg::Twist>("EST_v_c", 10);

        publisher_vo_pose_ = create_publisher<geometry_msgs::msg::Pose>("EST_vo_c", 10);

        publisher_acc_bias_ = create_publisher<geometry_msgs::msg::Pose>("Accel_bias_c", 10);

        // publisher_qua = create_publisher<geometry_msgs::msg::Quaternion>("qua", 10);

        time_0 = static_cast<double>(rclcpp::Clock().now().nanoseconds()) / 1e9;

        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0)
        {
            perror("socket creation failed");
            exit(EXIT_FAILURE);
        }

        memset(&servaddr1, 0, sizeof(servaddr1));

        servaddr1.sin_family = AF_INET;
        servaddr1.sin_port = htons(1234); // 第一个目标端口
        servaddr1.sin_addr.s_addr = inet_addr("127.0.0.1");
    }

    pogoxSub::~pogoxSub()
    {
        close(sockfd);
    }

    void pogoxSub::foot_imu_callback(const communication::msg::FootImu::SharedPtr msg)
    {

        // pogox_store.imu_time_ = static_cast<double>(msg.header.stamp.sec) + static_cast<double>(msg.header.stamp.nanosec) / 1e9 - time_0;
        rclcpp::Time imu_time = rclcpp::Clock().now();
        pogox_store->imu_time_ = static_cast<double>(imu_time.nanoseconds()) / 1e9 - time_0; // Correctly obtaining the timestamp in seconds as a double

        pogox_store->accel_b_(0) = msg->linear_acceleration.x;
        pogox_store->accel_b_(1) = msg->linear_acceleration.y;
        pogox_store->accel_b_(2) = msg->linear_acceleration.z;

        ngimu_accel_b(0) = msg->linear_acceleration.x;
        ngimu_accel_b(1) = msg->linear_acceleration.y;
        ngimu_accel_b(2) = msg->linear_acceleration.z;
        // pogox_store->quaternion_.x() = msg->orientation.x;
        // pogox_store->quaternion_.y() = msg->orientation.y;
        // pogox_store->quaternion_.z() = msg->orientation.z;
        // pogox_store->quaternion_.w() = msg->orientation.w;
        // std::cout << "imu" << std::endl;

        msg_num++;
    }

    void pogoxSub::vrpn_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

        pogox_store->pose_(0) = msg->pose.position.x;
        pogox_store->pose_(1) = msg->pose.position.y;
        pogox_store->pose_(2) = msg->pose.position.z;

        // pogox_store->quaternion_.x() = msg->pose.orientation.x;
        // pogox_store->quaternion_.y() = msg->pose.orientation.y;
        // pogox_store->quaternion_.z() = msg->pose.orientation.z;
        // pogox_store->quaternion_.w() = msg->pose.orientation.w;
        Quaterniond vrpn_quaternion;
        vrpn_quaternion.x() = msg->pose.orientation.x;
        vrpn_quaternion.y() = msg->pose.orientation.y;
        vrpn_quaternion.z() = msg->pose.orientation.z;
        vrpn_quaternion.w() = msg->pose.orientation.w;
        double yaw, pitch, roll = 0.0;
        quaternionToEuler(vrpn_quaternion, yaw, pitch, roll);
        vrpn_euler(0) = roll;
        vrpn_euler(1) = pitch;
        vrpn_euler(2) = yaw;

        if (!init_mocap)
        {
            mocap_quaternion_.x() = msg->pose.orientation.x;
            mocap_quaternion_.y() = msg->pose.orientation.y;
            mocap_quaternion_.z() = msg->pose.orientation.z;
            mocap_quaternion_.w() = msg->pose.orientation.w;

            init_mocap = true;
        }

        pose_ = pogox_store->pose_;
    }

    void pogoxSub::vrpn_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {

        pogox_store->v_sb_(0) = msg->linear.x;
        pogox_store->v_sb_(1) = msg->linear.y;
        pogox_store->v_sb_(2) = msg->linear.z;
        pose_v_ = pogox_store->v_sb_;
    }

    void pogoxSub::socket_callback(const custom_msgs::msg::SensorFloat::SharedPtr msg)
    {
        pogox_store->stance_ = msg->contact * 1.0;
        contact_ = msg->contact * 1.0;
        pogox_store->distance_ = msg->distance;
    }

    void pogoxSub::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {

        // pogox_store.imu_time_ = static_cast<double>(msg.header.stamp.sec) + static_cast<double>(msg.header.stamp.nanosec) / 1e9 - time_0;
        rclcpp::Time imu_time = rclcpp::Clock().now();
        pogox_store->imu_time_ = static_cast<double>(imu_time.nanoseconds()) / 1e9 - time_0; // Correctly obtaining the timestamp in seconds as a double

        pogox_store->accel_b_(0) = msg->linear_acceleration.x;
        pogox_store->accel_b_(1) = msg->linear_acceleration.y;
        pogox_store->accel_b_(2) = msg->linear_acceleration.z;
        // std::cout << "imu" << std::endl;
        ngimu_accel_b = pogox_store->accel_b_;

        msg_num++;
    }

    void pogoxSub::imu_filter_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {

        // lord_accel_b(0) = msg->linear_acceleration.x;
        // lord_accel_b(1) = msg->linear_acceleration.y;
        // lord_accel_b(2) = msg->linear_acceleration.z;
        pogox_store->quaternion_.x() = msg->orientation.x;
        pogox_store->quaternion_.y() = msg->orientation.y;
        pogox_store->quaternion_.z() = msg->orientation.z;
        pogox_store->quaternion_.w() = msg->orientation.w;

        double yaw, pitch, roll = 0.0;
        quaternionToEuler(pogox_store->quaternion_, yaw, pitch, roll);
        filter_euler(0) = roll;
        filter_euler(1) = pitch;
        filter_euler(2) = yaw;
    };

    void pogoxSub::orientation_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
    {
        // pogox_store->quaternion_.x() = msg->x;
        // pogox_store->quaternion_.y() = msg->y;
        // pogox_store->quaternion_.z() = msg->z;
        // pogox_store->quaternion_.w() = msg->w;
        // // R_sb_ = pogox_store->quaternion_.normalized().toRotationMatrix().inverse();
        // // Quaterniond inv = pogox_store.quaternion_.inverse();
        // // geometry_msgs::msg::Quaternion inv_q;
        // // inv_q.x = -inv.x();
        // // inv_q.y = -inv.y();
        // // inv_q.z = -inv.z();
        // // inv_q.w = -inv.w();
        // // publisher_qua->publish(inv_q);

        // if (init_mocap && !init_orientation)
        // {
        //     pogox_store->offset_quaternion_ = mocap_quaternion_ * pogox_store->quaternion_; // R_IMU_correct =  (R_Mocap * R_IMU^T) * R_IMU
        //     init_orientation = true;
        // }
    }

    void pogoxSub::mocap_callback(const optitrack_broadcast::msg::Mocap::SharedPtr msg)
    {
        pogox_store->pose_(0) = msg->position[0];
        pogox_store->pose_(1) = msg->position[1];
        pogox_store->pose_(2) = msg->position[2];

        pogox_store->v_sb_(0) = msg->velocity[0];
        pogox_store->v_sb_(1) = msg->velocity[1];
        pogox_store->v_sb_(2) = msg->velocity[2];

        pogox_store->quaternion_.x() = msg->quaternion[1];
        pogox_store->quaternion_.y() = msg->quaternion[2];
        pogox_store->quaternion_.z() = msg->quaternion[3];
        pogox_store->quaternion_.w() = msg->quaternion[0];

        if (!init_mocap)
        {
            mocap_quaternion_.x() = msg->quaternion[1];
            mocap_quaternion_.y() = msg->quaternion[2];
            mocap_quaternion_.z() = msg->quaternion[3];
            mocap_quaternion_.w() = msg->quaternion[0];
            init_mocap = true;
        }

        pose_v_ = pogox_store->v_sb_;
        pose_ = pogox_store->pose_;
    }

    void pogoxSub::vins_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        vins_pose_(0) = msg->pose.pose.position.x;
        vins_pose_(1) = msg->pose.pose.position.y;
        vins_pose_(2) = msg->pose.pose.position.z;

        vins_vel_(0) = msg->twist.twist.linear.x;
        vins_vel_(1) = msg->twist.twist.linear.y;
        vins_vel_(2) = msg->twist.twist.linear.z;

        if (!init_vins)
        {
            vins_quaternion_.x() = msg->pose.pose.orientation.x;
            vins_quaternion_.y() = msg->pose.pose.orientation.y;
            vins_quaternion_.z() = msg->pose.pose.orientation.z;
            vins_quaternion_.w() = msg->pose.pose.orientation.w;
            init_vins = true;
        }
        // vins_pose_ = vins_offset_.normalized().toRotationMatrix() * vins_pose_;
        // vins_vel_ = vins_offset_.normalized().toRotationMatrix() * vins_vel_;
    }

    void pogoxSub::contact_callback(const optitrack_broadcast::msg::SensorFloat::SharedPtr msg)
    {
        pogox_store->stance_ = msg->sensor;
        contact_ = msg->sensor;
    }

    void pogoxSub::lidar_callback(const optitrack_broadcast::msg::SensorFloat::SharedPtr msg)
    {
        pogox_store->distance_ = msg->sensor;
    }

    void pogoxSub::vo_callback(const orbslam3_msgs::msg::VoRealtiveTransform::SharedPtr msg)
    {
        // sub for the vo_pose_bpdy_pre_2_body,
        pogox_store->vo_new_ = true;
        pogox_store->vo_time_pre_ = static_cast<double>(msg->header_pre.stamp.sec) +
                                    static_cast<double>(msg->header_pre.stamp.nanosec) / 1e9 - time_0; // image_pre time stamp
        pogox_store->vo_time_now_ = static_cast<double>(msg->header.stamp.sec) +
                                    static_cast<double>(msg->header.stamp.nanosec) / 1e9 - time_0; // image_pre time stamp
        pogox_store->vo_pose_body_pre_2_body_(0) = msg->x_relative;
        pogox_store->vo_pose_body_pre_2_body_(1) = msg->y_relative;
        pogox_store->vo_pose_body_pre_2_body_(2) = msg->z_relative;

        vo_norm_ = pogox_store->vo_pose_body_pre_2_body_.norm();
        // pogox_store->vo_matches_ = msg->n_match;
        // vo_matches_ = msg->n_match * 1.0;
        // std::cout << "recieve vo: " << pogox_store->vo_time_pre_ << "||" << pogox_store->vo_time_now_ << std::endl;
        // std::cout << "diff: " << pogox_store->vo_time_now_ - pogox_store->imu_time_ << std::endl;
    }

    void pogoxSub::vo_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

        vo_pose(0) = msg->pose.position.x;
        vo_pose(1) = msg->pose.position.y;
        vo_pose(2) = msg->pose.position.z;

        vo_pose_quaternion_.x() = msg->pose.orientation.x;
        vo_pose_quaternion_.y() = msg->pose.orientation.y;
        vo_pose_quaternion_.z() = msg->pose.orientation.z;
        vo_pose_quaternion_.w() = msg->pose.orientation.w;

        vo_pose = vo_pose_offset_ * vo_pose;

        if (init_mocap && !init_vo_pose)
        {
            vo_pose_offset_ = mocap_quaternion_ * vo_pose_quaternion_.inverse(); // R_IMU_correct =  (R_Mocap * R_IMU^T) * R_IMU
            init_vo_pose = true;
        }
        // std::cout << "recieve vo: " << pogox_store->vo_time_pre_ << "||" << pogox_store->vo_time_now_ << std::endl;
        // std::cout << "diff: " << pogox_store->vo_time_now_ - pogox_store->imu_time_ << std::endl;
    }

    void pogoxSub::openvins_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        vins_vel_(0) = msg->twist.twist.linear.x;
        vins_vel_(1) = msg->twist.twist.linear.y;
        vins_vel_(2) = msg->twist.twist.linear.z;

        Quaterniond q_0;
        q_0 = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

        Quaterniond vins_quaternion;

        vins_quaternion.x() = -msg->pose.pose.orientation.x;
        vins_quaternion.y() = -msg->pose.pose.orientation.y;
        vins_quaternion.z() = -msg->pose.pose.orientation.z;
        vins_quaternion.w() = -msg->pose.pose.orientation.w;

        double yaw, pitch, roll = 0.0;
        quaternionToEuler(q_0 * vins_quaternion, yaw, pitch, roll);
        vins_euler(0) = roll;
        vins_euler(1) = pitch;
        vins_euler(2) = yaw;
    }
    void pogoxSub::timerCallback()
    {
        auto start1 = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count();

        if (msg_num >= 10)
        {

            if (discrete_time == 0)
            {
                mhe.initialize(pogox_store, robot_params_);

                pathx = pogox_store->pose_(0);
                pathy = pogox_store->pose_(1);
                pathz = pogox_store->pose_(2);
            }
            else
            {
                mhe.update(discrete_time);
                // pose_(0) += -pathx;
                // pose_(1) += -pathy;
                // pose_(2) += -pathz;
            }
            discrete_time++;

            // visualization and logging
            if (this->get_parameter("visualize").as_int())
            {
                visualize();
            }
            if (discrete_time == robot_params_->N_ + 1)
            {
                init_logging();
            }
            if (discrete_time > robot_params_->N_ + 1)
            {
                vo_norm_used_ = mhe.vo_pose_body_pre_2_body_.norm();

                logger.spin_logging();
            }

            auto start = std::chrono::system_clock::now();

            // Convert to time since epoch as a double
            auto start_time_t = std::chrono::duration<double>(start.time_since_epoch()).count();

            // FIFO string
            std::stringstream ss;
            double num1 = mhe.x_MHE(3);
            double num2 = mhe.x_MHE(4);
            double num3 = mhe.x_MHE(5);

            ss << start_time_t - floor(start_time_t) << " " << num1 << " " << num2 << " " << num3 << " " << discrete_time << " ";
            std::string data = ss.str();

            if (sendto(sockfd, data.c_str(), data.length(), 0, (const struct sockaddr *)&servaddr1, sizeof(servaddr1)) < 0)
            {
                perror("sendto failed");
            }
            else
            {
                std::cout << "UDP message sent to port 1234: " << data << std::endl;
            }
        }
        auto stop1 = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count();
        auto duration1 = static_cast<double>(stop1 - start1) / 1000000;
        // std::cout << "Total loop hz elapsed: " << 1 / duration1 << " hz." << std::endl;
        std::cout << 1 / duration1 << std::endl;
    }

    void pogoxSub::init_logging()
    {
        std::string log_name = "mhe_" + this->get_parameter("log_name").as_string();
        if (est_type_ == 0)
        {
            logger.init(log_name);
            logger.add_data(pose_, "pose");
            logger.add_data(pose_v_, "GT_v");
            logger.add_data(mhe.x_MHE, "X_MHE");
            logger.add_data(mhe.x_body_accmulate, "vo_traj");
            logger.add_data(lord_accel_b, "accel_s_");
            logger.add_data(ngimu_accel_b, "accel_s_");
            logger.add_data(&contact_, "contact_");
            logger.add_data(vins_pose_, "vins_pose_");
            logger.add_data(vins_vel_, "vins_vel_");
            logger.add_data(vo_pose, "vo_pose");
            logger.add_data(&vo_norm_, "vo_norm_");
            logger.add_data(&vo_norm_used_, "vo_norm_used_");
            logger.add_data(filter_euler, "filter");
            logger.add_data(vrpn_euler, "vrpn");
            logger.add_data(vins_euler, "vins");

            // logger.add_data(&mhe.Contact_, "Contact_");
            // logger.add_data(mhe.x_MHE, "x_solution_now");
        }
        if (est_type_ == 1)
        {
            // logger.init("kf_est");

            // // logger.add_data(pogox_store.pose_, "pose");
            // logger.add_data(pose_v_, "pose_v");
            // logger.add_data(mhe.x_KF, "X_KF");
            // logger.add_data(velocity_b, "velocity_b");
        }
    }

    void pogoxSub::quaternionToEuler(Quaterniond quaternion, double &yaw, double &pitch, double &roll)
    {
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
    }

    void pogoxSub::visualize()
    {
        if (msg_num > 1 && discrete_time != 1)
        {

            geometry_msgs::msg::Pose Pose_GT;
            Pose_GT.position.x = pose_(0) - pathx;
            Pose_GT.position.y = pose_(1) - pathy;
            Pose_GT.position.z = pose_(2) - pathz;
            publisher_GT_base_->publish(Pose_GT);

            geometry_msgs::msg::Twist twist_EST;
            // velocity_b = mhe.R_sb_.transpose() * mhe.x_MHE.segment(3, 3);
            Vector3d velocity_s = mhe.x_MHE.segment(3, 3);
            twist_EST.linear.x = velocity_s(0);
            twist_EST.linear.y = velocity_s(1);
            twist_EST.linear.z = velocity_s(2);
            publisher_velocity_EST_->publish(twist_EST);

            geometry_msgs::msg::Pose pose_EST_start;
            pose_EST_start.position.x = mhe.x_body_accmulate(0);
            pose_EST_start.position.y = mhe.x_body_accmulate(1);
            pose_EST_start.position.z = mhe.x_body_accmulate(2);
            publisher_vo_pose_->publish(pose_EST_start);

            geometry_msgs::msg::Pose Pose_EST;
            Vector3d pose_b = mhe.x_MHE.segment(0, 3);
            Pose_EST.position.x = pose_b(0);
            Pose_EST.position.y = pose_b(1);
            Pose_EST.position.z = pose_b(2);
            publisher_EST_base_->publish(Pose_EST);

            geometry_msgs::msg::Twist twist_GT;
            twist_GT.linear.x = pose_v_(0);
            twist_GT.linear.y = pose_v_(1);
            twist_GT.linear.z = pose_v_(2);
            publisher_velocity_GT_->publish(twist_GT);

            geometry_msgs::msg::Pose Accel_bias;
            Vector3d accel_bias_s_ = mhe.x_MHE.segment(9, 3);
            Accel_bias.position.x = accel_bias_s_(0);
            Accel_bias.position.y = accel_bias_s_(1);
            Accel_bias.position.z = accel_bias_s_(2);
            publisher_acc_bias_->publish(Accel_bias);
        }
    }

    void pogoxSub::paramsWrapper()
    {
        this->declare_parameter("p_process_std", std::vector<double>{0.01, 0.01, 0.01});
        this->declare_parameter("accel_input_std", std::vector<double>{0.01, 0.04, 0.001});
        this->declare_parameter("accel_input_contact_std", std::vector<double>{5.5, 5.5, 5.5});
        this->declare_parameter("accel_input_impact_std", std::vector<double>{5.5, 5.5, 5.5});

        this->declare_parameter("accel_bias_process_std", std::vector<double>{1., 1., 0.1});

        this->declare_parameter("foothold_slide_std", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("foot_swing_std", 1000000000.0);

        this->declare_parameter("lidar_std", std::vector<double>{0.1, 0.1, 0.05});
        this->declare_parameter("vo_pose_std", std::vector<double>{0.001, 0.001, 0.001});

        this->declare_parameter("p_init_std", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("v_init_std", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("foot_init_std", 0.025);
        this->declare_parameter("accel_bias_init_std", 0.1);

        this->declare_parameter("lidar_z_std", 0.05);

        this->declare_parameter("rate", 50);
        this->declare_parameter("N", 50);
        this->declare_parameter("est_type", 0);

        this->declare_parameter("interval", 20);

        this->declare_parameter("rho", 0.1);
        this->declare_parameter("alpha", 1.6);
        this->declare_parameter("delta", 0.00001);
        this->declare_parameter("sigma", 0.00001);
        this->declare_parameter("verbose", true);
        this->declare_parameter("adaptRho", true);
        this->declare_parameter("polish", true);
        this->declare_parameter("maxQPIter", 1000);
        this->declare_parameter("realtiveTol", 1e-3);
        this->declare_parameter("absTol", 1e-3);

        this->declare_parameter("primTol", 0.000001);
        this->declare_parameter("dualTol", 0.000001);
        this->declare_parameter("timeLimit", 0.005);

        this->declare_parameter("visualize", 0);

        this->declare_parameter<std::string>("log_name", "exp");
        // Retrieve parameters
        robot_params_->p_process_std_ = this->get_parameter("p_process_std").as_double_array();
        robot_params_->accel_input_std_ = this->get_parameter("accel_input_std").as_double_array();
        robot_params_->accel_input_contact_std_ = this->get_parameter("accel_input_contact_std").as_double_array();
        robot_params_->accel_input_impact_std_ = this->get_parameter("accel_input_impact_std").as_double_array();

        robot_params_->accel_bias_std_ = this->get_parameter("accel_bias_process_std").as_double_array();

        robot_params_->foothold_slide_std_ = this->get_parameter("foothold_slide_std").as_double_array();
        robot_params_->foot_swing_std_ = this->get_parameter("foot_swing_std").as_double();

        robot_params_->lidar_std_ = this->get_parameter("lidar_std").as_double_array();
        robot_params_->vo_pose_std_ = this->get_parameter("vo_pose_std").as_double_array();
        robot_params_->lidar_z_std_ = this->get_parameter("lidar_z_std").as_double();

        robot_params_->p_init_std_ = this->get_parameter("p_init_std").as_double_array();
        robot_params_->v_init_std_ = this->get_parameter("v_init_std").as_double_array();
        robot_params_->foot_init_std_ = this->get_parameter("foot_init_std").as_double();
        robot_params_->accel_bias_init_std_ = this->get_parameter("accel_bias_init_std").as_double();

        robot_params_->rate_ = this->get_parameter("rate").as_int();
        robot_params_->N_ = this->get_parameter("N").as_int();

        robot_params_->est_type_ = this->get_parameter("est_type").as_int();
        robot_params_->timeLimit_ = this->get_parameter("timeLimit").as_double();

        // osqp parameters
        robot_params_->rho_ = this->get_parameter("rho").as_double();
        robot_params_->alpha_ = this->get_parameter("alpha").as_double();
        robot_params_->primTol_ = this->get_parameter("primTol").as_double();
        robot_params_->dualTol_ = this->get_parameter("dualTol").as_double();
        robot_params_->delta_ = this->get_parameter("delta").as_double();
        robot_params_->sigma_ = this->get_parameter("sigma").as_double();

        robot_params_->verbose_ = this->get_parameter("verbose").as_bool();
        robot_params_->adaptRho_ = this->get_parameter("adaptRho").as_bool();
        robot_params_->polish_ = this->get_parameter("polish").as_bool();
        robot_params_->maxQPIter_ = this->get_parameter("maxQPIter").as_int();
        robot_params_->realtiveTol_ = this->get_parameter("realtiveTol").as_double();
        robot_params_->absTol_ = this->get_parameter("absTol").as_double();
    }
}
