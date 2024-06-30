#include "decentral_legged_est/EstSub.hpp"

using namespace Eigen;

namespace robotSub
{

    robotSub::robotSub(const std::string &name) : Node(name)
    {
        robot_store_ = std::make_shared<robot_store>();   // measuremenst struc ptr
        robot_params_ = std::make_shared<robot_params>(); // params struc ptr

        paramsWrapper();
        int timer_interval_ = this->get_parameter("estimation.interval").as_int();

        // Decentralized Orientation
        orien_filter_sub = create_subscription<sensor_msgs::msg::Imu>("imu/filter",
                                                                      10,
                                                                      std::bind(&robotSub::orien_filter_callback, this, std::placeholders::_1));
        // Sparsly integrated VO
        vo_sub = create_subscription<custom_msgs::msg::VoRealtiveTransform>("orb/vo",
                                                                            10,
                                                                            std::bind(&robotSub::vo_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(std::chrono::milliseconds(timer_interval_), std::bind(&robotSub::timerCallback, this));

        time_init_ = static_cast<double>(rclcpp::Clock().now().nanoseconds()) / 1e9;
    }

    robotSub::~robotSub()
    {
    }

    void robotSub::orien_filter_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Decentralized orientation callback
        robot_store_->quaternion_.x() = msg->orientation.x;
        robot_store_->quaternion_.y() = msg->orientation.y;
        robot_store_->quaternion_.z() = msg->orientation.z;
        robot_store_->quaternion_.w() = msg->orientation.w;

        EigenUtils::QuaternionToEuler(robot_store_->quaternion_, filter_euler_);
    }
    
    void robotSub::vo_callback(const custom_msgs::msg::VoRealtiveTransform::SharedPtr msg)
    {
        // Sparsly integrated VO callback; vo_p_body_pre_2_body: relative translation from body_pre to body
        robot_store_->vo_new_ = true;
        robot_store_->vo_time_pre_ = static_cast<double>(msg->header_pre.stamp.sec) +
                                     static_cast<double>(msg->header_pre.stamp.nanosec) / 1e9 - time_init_; // image_pre time stamp
        robot_store_->vo_time_now_ = static_cast<double>(msg->header.stamp.sec) +
                                     static_cast<double>(msg->header.stamp.nanosec) / 1e9 - time_init_; // image time stamp
        robot_store_->vo_p_body_pre_2_body_(0) = msg->x_relative;
        robot_store_->vo_p_body_pre_2_body_(1) = msg->y_relative;
        robot_store_->vo_p_body_pre_2_body_(2) = msg->z_relative;
    }

    void robotSub::timerCallback()
    {
        auto start_callback = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count();

        if (imu_msg_num_ >= 10)
        {

            if (discrete_time_ == 0)
            {
                mhe.initialize(robot_store_, robot_params_);
                gt_p_offset_ = robot_store_->gt_p_;
            }
            else
            {
                mhe.update(discrete_time_);
            }
            discrete_time_++;

            // logging
            if (discrete_time_ == robot_params_->N_ + 1)
            {
                init_logging();
            }
            if (discrete_time_ > robot_params_->N_ + 1)
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
            std::string log_name = this->get_parameter("log_name").as_string();

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
            std::string log_name =  this->get_parameter("log_name").as_string();

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

        // prior params
        this->declare_parameter("prior.p_init_std", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("prior.v_init_std", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("prior.foot_init_std", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("prior.accel_bias_init_std", std::vector<double>{0.001, 0.001, 0.001});
        robot_params_->p_init_std_ = this->get_parameter("prior.p_init_std").as_double_array();
        robot_params_->v_init_std_ = this->get_parameter("prior.v_init_std").as_double_array();
        robot_params_->foot_init_std_ = this->get_parameter("prior.foot_init_std").as_double_array();
        robot_params_->accel_bias_init_std_ = this->get_parameter("prior.accel_bias_init_std").as_double_array();

        // process params
        this->declare_parameter("process.p_process_std", std::vector<double>{0.01, 0.01, 0.01});
        this->declare_parameter("process.accel_input_std", std::vector<double>{0.01, 0.04, 0.001});
        this->declare_parameter("process.gyro_input_std", std::vector<double>{0.01, 0.01, 0.01});
        this->declare_parameter("process.accel_bias_process_std", std::vector<double>{1., 1., 0.1});
        robot_params_->p_process_std_ = this->get_parameter("process.p_process_std").as_double_array();
        robot_params_->accel_input_std_ = this->get_parameter("process.accel_input_std").as_double_array();
        robot_params_->gyro_input_std_ = this->get_parameter("process.gyro_input_std").as_double_array();
        robot_params_->accel_bias_std_ = this->get_parameter("process.accel_bias_process_std").as_double_array();

        // LO params
        this->declare_parameter("leg_odom.quaternion_ib", std::vector<double>{1.0, 0.0, 0.0, 0.0});
        this->declare_parameter("leg_odom.p_ib", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter("leg_odom.num_leg", 4);
        this->declare_parameter("leg_odom.leg_odom_type", 0);
        this->declare_parameter("leg_odom.joint_position_std", std::vector<double>{0.01, 0.01, 0.01});
        this->declare_parameter("leg_odom.joint_velocity_std", std::vector<double>{0.01, 0.01, 0.01});
        this->declare_parameter("leg_odom.foot_slide_std", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("leg_odom.foot_swing_std", std::vector<double>{10000.0, 10000.0, 10000.0});
        this->declare_parameter("leg_odom.contact_effort_theshold", 150.0);

        robot_params_->quaternion_ib_ = this->get_parameter("leg_odom.quaternion_ib").as_double_array();
        robot_params_->p_ib_ = this->get_parameter("leg_odom.p_ib").as_double_array();
        robot_params_->num_legs_ = this->get_parameter("leg_odom.num_leg").as_int();
        robot_params_->leg_odom_type_ = this->get_parameter("leg_odom.leg_odom_type").as_int();
        robot_params_->joint_position_std_ = this->get_parameter("leg_odom.joint_position_std").as_double_array();
        robot_params_->joint_velocity_std_ = this->get_parameter("leg_odom.joint_velocity_std").as_double_array();
        robot_params_->foot_slide_std_ = this->get_parameter("leg_odom.foot_slide_std").as_double_array();
        robot_params_->foot_swing_std_ = this->get_parameter("leg_odom.foot_swing_std").as_double_array();
        robot_params_->contact_effort_theshold_ = this->get_parameter("leg_odom.contact_effort_theshold").as_double();

        // vo params
        this->declare_parameter("visual_odom.vo_p_std", std::vector<double>{0.001, 0.001, 0.001});
        robot_params_->vo_p_std_ = this->get_parameter("visual_odom.vo_p_std").as_double_array();

        // estimation params
        this->declare_parameter("estimation.rate", 50);
        this->declare_parameter("estimation.interval", 20);
        this->declare_parameter("estimation.N", 50);
        this->declare_parameter("estimation.est_type", 0);
        robot_params_->rate_ = this->get_parameter("estimation.rate").as_int();
        robot_params_->N_ = this->get_parameter("estimation.N").as_int();
        robot_params_->est_type_ = this->get_parameter("estimation.est_type").as_int();

        // osqp params
        this->declare_parameter("osqp.rho", 0.1);
        this->declare_parameter("osqp.alpha", 1.6);
        this->declare_parameter("osqp.delta", 0.00001);
        this->declare_parameter("osqp.sigma", 0.00001);
        this->declare_parameter("osqp.verbose", true);
        this->declare_parameter("osqp.adaptRho", true);
        this->declare_parameter("osqp.polish", true);
        this->declare_parameter("osqp.maxQPIter", 1000);
        this->declare_parameter("osqp.primTol", 0.000001);
        this->declare_parameter("osqp.dualTol", 0.000001);
        this->declare_parameter("osqp.realtiveTol", 1e-3);
        this->declare_parameter("osqp.absTol", 1e-3);
        this->declare_parameter("osqp.timeLimit", 0.005);
        robot_params_->rho_ = this->get_parameter("osqp.rho").as_double();
        robot_params_->alpha_ = this->get_parameter("osqp.alpha").as_double();
        robot_params_->delta_ = this->get_parameter("osqp.delta").as_double();
        robot_params_->sigma_ = this->get_parameter("osqp.sigma").as_double();
        robot_params_->verbose_ = this->get_parameter("osqp.verbose").as_bool();
        robot_params_->adaptRho_ = this->get_parameter("osqp.adaptRho").as_bool();
        robot_params_->polish_ = this->get_parameter("osqp.polish").as_bool();
        robot_params_->maxQPIter_ = this->get_parameter("osqp.maxQPIter").as_int();
        robot_params_->primTol_ = this->get_parameter("osqp.primTol").as_double();
        robot_params_->dualTol_ = this->get_parameter("osqp.dualTol").as_double();
        robot_params_->realtiveTol_ = this->get_parameter("osqp.realtiveTol").as_double();
        robot_params_->absTol_ = this->get_parameter("osqp.absTol").as_double();
        robot_params_->timeLimit_ = this->get_parameter("osqp.timeLimit").as_double();
    }
}
