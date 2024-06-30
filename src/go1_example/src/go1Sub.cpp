#include "go1Sub.hpp"

using namespace Eigen;

namespace robotSub
{

    go1Sub::go1Sub(const std::string &name) : robotSub(name)
    {

        // Go1 sensors
        // IMU 
        imu_sub = create_subscription<sensor_msgs::msg::Imu>("/unitree/imu",
                                                             10,
                                                             std::bind(&go1Sub::imu_callback, this, std::placeholders::_1));
        //Leg Odometry, encoder information & kinematics & contact 
        lo_sub = create_subscription<sensor_msgs::msg::JointState>("/unitree/joint_state",
                                                                   10,
                                                                   std::bind(&go1Sub::lo_callback, this, std::placeholders::_1));
        // Ground Truth
        mocap_sub = create_subscription<optitrack_broadcast::msg::Mocap>("/mocap/RigidBody",
                                                                         10,
                                                                         std::bind(&go1Sub::mocap_callback, this, std::placeholders::_1));
    }

    go1Sub::~go1Sub()
    {
    }

    void go1Sub::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        //---------------------------------------------------------------
        // ToDo: configurate/fill-up the imu_time, accel_b_, angular_b_;
        // robot_store_->imu_time_: double;
        // robot_store_->accel_b_: Vector3d;
        // robot_store_->angular_b_: Vector3d;
        //---------------------------------------------------------------

        rclcpp::Time imu_time = rclcpp::Clock().now();
        robot_store_->imu_time_ = static_cast<double>(imu_time.nanoseconds()) / 1e9 - time_init_; // Correctly obtaining the timestamp in seconds as a double

        robot_store_->accel_b_(0) = msg->linear_acceleration.x;
        robot_store_->accel_b_(1) = msg->linear_acceleration.y;
        robot_store_->accel_b_(2) = msg->linear_acceleration.z;

        robot_store_->angular_b_(0) = msg->angular_velocity.x;
        robot_store_->angular_b_(1) = msg->angular_velocity.y;
        robot_store_->angular_b_(2) = msg->angular_velocity.z;

        imu_msg_num_++;
    }

    void go1Sub::lo_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        //---------------------------------------------------------------
        // ToDo: configurate/fill-up the encoder_position, encoder_velocity, foot_forward kinematics, foot_jacobian, contact;
        // robot_store_->joint_states_position_: VectorXd;
        // robot_store_->joint_states_velocity_: VectorXd;
        // robot_store_->p_imu_2_foot_: MatrixXd, (3*num_legs,1);
        // robot_store_->J_imu_2_foot_: MatrixXd, (3*num_legs,3);
        // robot_store_->contact_ = VectorXd (0: no_contact, 1: contact);
        //---------------------------------------------------------------

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
        
        // Foot order: FR, FL, RR, RL; for both hardware and kinematics lib
        // Joints Order: hip, thig, calf, foot(fixed 0); for both hardware (no foot joint) and kinematics lib
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

    void go1Sub::mocap_callback(const optitrack_broadcast::msg::Mocap::SharedPtr msg)
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
        gt_quaternion_.x() = msg->quaternion[1];
        gt_quaternion_.y() = msg->quaternion[2];
        gt_quaternion_.z() = msg->quaternion[3];
        gt_quaternion_.w() = msg->quaternion[0];

        EigenUtils::QuaternionToEuler(gt_quaternion_, gt_euler_);

        gt_p_ = robot_store_->gt_p_ - gt_p_offset_;
        gt_v_b_ = gt_quaternion_.normalized().toRotationMatrix() * robot_store_->gt_v_s_;
    }
}
int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<robotSub::go1Sub>("est_sub");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
