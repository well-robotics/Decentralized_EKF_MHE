#include "orien_ekf.hpp"

using namespace Eigen;

namespace orien_ekf
{

    orien_ekf::orien_ekf(const std::string &name) : Node(name)
    {

        gravity_ << 0, 0, 9.81;
        C_gyro_.diagonal() << std::pow(0.1, 2), std::pow(0.1, 2), std::pow(0.1, 2);
        C_accel_.diagonal() << std::pow(4.0, 2), std::pow(4.0, 2), std::pow(4.0, 2);
        C_vo_.diagonal() << std::pow(0.0001, 2), std::pow(0.0001, 2), std::pow(0.0001, 2), std::pow(0.0001, 2);

        quternion_prior_ = VectorXd::Zero(4);
        quternion_prior_(0) = 1;

        quaternion_ = quternion_prior_;
        Cov_q_ = 0.001 * 0.001 * MatrixXd::Identity(4, 4);

        vo_pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>("orb/pos",
                                                                           10,
                                                                           std::bind(&orien_ekf::vo_pose_callback, this, std::placeholders::_1));
        imu_sub = create_subscription<sensor_msgs::msg::Imu>("unitree/imu",
                                                             10,
                                                             std::bind(&orien_ekf::imu_callback, this, std::placeholders::_1));
        // imu_ros2_sub = create_subscription<communication::msg::FootImu>("/hardware/ngimu_sensor_ACM1",
        //                                                                 10,
        //                                                                 std::bind(&orien_ekf::imu_ros2_callback, this, std::placeholders::_1));

        vrpn_sub = create_subscription<geometry_msgs::msg::PoseStamped>("/vrpn_mocap/RigidBody/pose",
                                                                        10,
                                                                        std::bind(&orien_ekf::vrpn_callback, this, std::placeholders::_1));
        mocap_sub = create_subscription<optitrack_broadcast::msg::Mocap>("/mocap/RigidBody",
                                                                         10,
                                                                         std::bind(&orien_ekf::mocap_callback, this, std::placeholders::_1));
        publisher_filter_ = create_publisher<sensor_msgs::msg::Imu>("imu/filter", 10);
        publisher_filter_euler_ = create_publisher<geometry_msgs::msg::Vector3>("imu/filter/euler", 10);
        publisher_mocap_euler_ = create_publisher<geometry_msgs::msg::Vector3>("mocap/euler", 10);
        publisher_vo_euler_ = create_publisher<geometry_msgs::msg::Vector3>("vo/euler", 10);

        timer_ = create_wall_timer(std::chrono::microseconds(2000), std::bind(&orien_ekf::timerCallback, this));
        time_0_ = static_cast<double>(rclcpp::Clock().now().nanoseconds()) / 1e9;
    }

    void orien_ekf::vo_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        vo_time_ = static_cast<double>(msg->header.stamp.sec) +
                   static_cast<double>(msg->header.stamp.nanosec) / 1e9 - time_0_; // image_pre time stamp
        vo_pose_quaternion_(1) = msg->pose.orientation.x;
        vo_pose_quaternion_(2) = msg->pose.orientation.y;
        vo_pose_quaternion_(3) = msg->pose.orientation.z;
        vo_pose_quaternion_(0) = msg->pose.orientation.w;
        vo_new_ = true;
        init_vo = 1;

        double roll, pitch, yaw = 0.0;

        quaternionToEuler(vo_pose_quaternion_, yaw, pitch, roll);
        geometry_msgs::msg::Vector3 vo_euler;
        vo_euler.x = roll;
        vo_euler.y = pitch;
        vo_euler.z = yaw;
        publisher_vo_euler_->publish(vo_euler);
    }

    void orien_ekf::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {

        imu_time_ = static_cast<double>(rclcpp::Clock().now().nanoseconds()) / 1e9 - time_0_; // Correctly obtaining the timestamp in seconds as a double

        accel_b_(0) = msg->linear_acceleration.x;
        accel_b_(1) = msg->linear_acceleration.y;
        accel_b_(2) = msg->linear_acceleration.z;

        angular_vel_b_(0) = msg->angular_velocity.x;
        angular_vel_b_(1) = msg->angular_velocity.y;
        angular_vel_b_(2) = msg->angular_velocity.z;

        init_imu = 1;
    }

    // void orien_ekf::imu_ros2_callback(const communication::msg::FootImu::SharedPtr msg)
    // {
    //     imu_header.stamp = msg->header.stamp;

    //     imu_time_ = static_cast<double>(rclcpp::Clock().now().nanoseconds()) / 1e9 - time_0_; // Correctly obtaining the timestamp in seconds as a double

    //     accel_b_(0) = msg->linear_acceleration.x;
    //     accel_b_(1) = msg->linear_acceleration.y;
    //     accel_b_(2) = msg->linear_acceleration.z;

    //     angular_vel_b_(0) = msg->angular_velocity.x;
    //     angular_vel_b_(1) = msg->angular_velocity.y;
    //     angular_vel_b_(2) = msg->angular_velocity.z;

    //     init_imu = 1;
    // }
    void orien_ekf::vrpn_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

        mocap_quaternion_(1) = msg->pose.orientation.x;
        mocap_quaternion_(2) = msg->pose.orientation.y;
        mocap_quaternion_(3) = msg->pose.orientation.z;
        mocap_quaternion_(0) = msg->pose.orientation.w;

        double roll, pitch, yaw = 0.0;

        quaternionToEuler(mocap_quaternion_, yaw, pitch, roll);
        geometry_msgs::msg::Vector3 mocap_euler;
        mocap_euler.x = roll;
        mocap_euler.y = pitch;
        mocap_euler.z = yaw;
        publisher_mocap_euler_->publish(mocap_euler);

        init_mocap = 1;
    }
    void orien_ekf::mocap_callback(const optitrack_broadcast::msg::Mocap::SharedPtr msg)
    {

        mocap_quaternion_(1) = msg->quaternion[1];
        mocap_quaternion_(2) = msg->quaternion[2];
        mocap_quaternion_(3) = msg->quaternion[3];
        mocap_quaternion_(0) = msg->quaternion[0];

        double roll, pitch, yaw = 0.0;

        quaternionToEuler(mocap_quaternion_, yaw, pitch, roll);
        geometry_msgs::msg::Vector3 mocap_euler;
        mocap_euler.x = roll;
        mocap_euler.y = pitch;
        mocap_euler.z = yaw;
        publisher_mocap_euler_->publish(mocap_euler);

        init_mocap = 1;
    }
    void orien_ekf::timerCallback()
    {
        // auto start1 = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count();
        if (!init_ && init_mocap * init_imu)
        {
            quaternion_ = mocap_quaternion_;
            init_ = 1;
        }
        if (init_imu && init_)
        {
            get_measurement();
            gyro_nonlinear_predict(quaternion_pred_, quaternion_, angular_vel_b_, Cov_q_, Cov_q_pred_);
            gyro_nonlinear_correct(quaternion_correct_, quaternion_pred_, accel_b_, Cov_q_pred_, Cov_q_correct_);

            quaternion_ = quaternion_correct_;
            Cov_q_ = Cov_q_correct_;

            discrete_time++;

            Quaterniond mocap;
            Quaterniond current;
            mocap.w() = mocap_quaternion_(0);
            mocap.x() = mocap_quaternion_(1);
            mocap.y() = mocap_quaternion_(2);
            mocap.z() = mocap_quaternion_(3);

            current.w() = quaternion_(0);
            current.x() = quaternion_(1);
            current.y() = quaternion_(2);
            current.z() = quaternion_(3);

            Quaterniond offset = mocap.inverse() * current;

            // std::cout << offset.w() << "||" << offset.x() << "||" << offset.y() << "||" << offset.z() << std::endl;
        }
        double roll, pitch, yaw = 0.0;

        quaternionToEuler(quaternion_, yaw, pitch, roll);

        geometry_msgs::msg::Vector3 imu_pub_euler;
        imu_pub_euler.x = roll;
        imu_pub_euler.y = pitch;
        imu_pub_euler.z = yaw;
        publisher_filter_euler_->publish(imu_pub_euler);

        sensor_msgs::msg::Imu imu_pub_msg;
        imu_pub_msg.header.stamp = imu_header.stamp;
        imu_pub_msg.orientation.w = quaternion_(0);
        imu_pub_msg.orientation.x = quaternion_(1);
        imu_pub_msg.orientation.y = quaternion_(2);
        imu_pub_msg.orientation.z = quaternion_(3);

        imu_pub_msg.linear_acceleration.x = accel_b_(0);
        imu_pub_msg.linear_acceleration.y = accel_b_(1);
        imu_pub_msg.linear_acceleration.z = accel_b_(2);

        imu_pub_msg.angular_velocity.x = angular_vel_b_(0);
        imu_pub_msg.angular_velocity.y = angular_vel_b_(1);
        imu_pub_msg.angular_velocity.z = angular_vel_b_(2);

        publisher_filter_->publish(imu_pub_msg);
        // auto stop1 = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count();
        // auto duration1 = static_cast<double>(stop1 - start1) / 1000000;
        // // std::cout << "Total loop hz elapsed: " << 1 / duration1 << " hz." << std::endl;
        // std::cout << 1 / duration1 << std::endl;
    }

    void orien_ekf::gyro_nonlinear_predict(VectorXd &quaternion_pred, VectorXd &quaternion, Vector3d &gyro_reading, MatrixXd &Cov_q, MatrixXd &Cov_q_pred)
    {
        MatrixXd Ohm = MatrixXd::Zero(4, 4);
        gyro_2_Ohm(gyro_reading, Ohm);

        MatrixXd W = MatrixXd::Zero(4, 3);
        quat_2_W(quaternion, W);

        MatrixXd F = MatrixXd::Identity(4, 4) + dt_ / 2 * Ohm;

        quaternion_pred = F * quaternion;

        Cov_q_pred = F * Cov_q * F.transpose() + W * C_gyro_ * W.transpose();

        quat_norm(quaternion_pred);
    }

    void orien_ekf::gyro_nonlinear_correct(VectorXd &quaternion_correct, VectorXd &quaternion_pred, Vector3d &accel_readings, MatrixXd &Cov_q_pred, MatrixXd &Cov_q_correct)
    {
        MatrixXd Rotation_pred = MatrixXd::Zero(3, 3);
        quat_2_Rot(quaternion_pred, Rotation_pred);

        Vector3d accel_hat = Rotation_pred.transpose() * gravity_;

        MatrixXd H = MatrixXd::Zero(3, 4);
        quat_2_H(quaternion_pred, H);

        double accel_relative_norm = accel_readings.norm() / gravity_.norm();

        MatrixXd K = Cov_q_pred * H.transpose() * (H * Cov_q_pred * H.transpose() + accel_relative_norm * accel_relative_norm * C_accel_).inverse();
        quaternion_correct = quaternion_pred + K * (accel_readings - accel_hat);
        // std::cout << accel_readings - accel_hat << std::endl;
        Cov_q_correct = (MatrixXd::Identity(4, 4) - K * H) * Cov_q_pred;

        quat_norm(quaternion_correct);
    }

    void orien_ekf::vo_nonlinear_correct(VectorXd &quaternion_correct, VectorXd &quaternion_pred, VectorXd &quaternion_vo, MatrixXd &Cov_q_pred, MatrixXd &Cov_q_correct)
    {

        MatrixXd H = MatrixXd::Identity(4, 4);

        MatrixXd K = Cov_q_pred * H.transpose() * (H * Cov_q_pred * H.transpose() + C_vo_).inverse();
        quaternion_correct = quaternion_pred + K * (quaternion_vo - quaternion_pred);
        Cov_q_correct = (MatrixXd::Identity(4, 4) - K * H) * Cov_q_pred;

        quat_norm(quaternion_correct);
    }

    void orien_ekf::gyro_2_Ohm(Vector3d &gyro_reading, MatrixXd &Ohm)
    {
        // quaternion = [w,x,y,z];
        Ohm.block<1, 3>(0, 1) << -gyro_reading.transpose();

        Ohm.block<3, 1>(1, 0) << gyro_reading;

        Ohm(1, 2) = gyro_reading(2);
        Ohm(1, 3) = -gyro_reading(1);
        Ohm(2, 3) = gyro_reading(0);

        Ohm(2, 1) = -gyro_reading(2);
        Ohm(3, 1) = gyro_reading(1);
        Ohm(3, 2) = -gyro_reading(0);
    }

    void orien_ekf::quat_inv(VectorXd &quaternion, VectorXd &quaternion_out)
    {

        Quaterniond q;
        q.w() = quaternion(0);
        q.x() = quaternion(1);
        q.y() = quaternion(2);
        q.z() = quaternion(3);
        Quaterniond q_inv;
        q_inv = q.normalized().inverse();
        quaternion_out(0) = q_inv.w();
        quaternion_out(1) = q_inv.x();
        quaternion_out(2) = q_inv.y();
        quaternion_out(3) = q_inv.z();
    }

    void orien_ekf::quat_mul(VectorXd &quaternion_left, VectorXd &quaternion_right, VectorXd &quaternion_output)
    {

        Quaterniond q_left;
        q_left.w() = quaternion_left(0);
        q_left.x() = quaternion_left(1);
        q_left.y() = quaternion_left(2);
        q_left.z() = quaternion_left(3);
        Quaterniond q_right;
        q_right.w() = quaternion_right(0);
        q_right.x() = quaternion_right(1);
        q_right.y() = quaternion_right(2);
        q_right.z() = quaternion_right(3);

        Quaterniond q_out;
        q_out = q_left * q_right;
        q_out = q_out.normalized();

        quaternion_output(0) = q_out.w();
        quaternion_output(1) = q_out.x();
        quaternion_output(2) = q_out.y();
        quaternion_output(3) = q_out.z();
    }

    void orien_ekf::quat_2_W(VectorXd &quaternion, MatrixXd &W)
    {
        // quaternion = [w,x,y,z];
        // W = dt /2 *[-x -y -z;
        //      w  -z  y;
        //      z   w -x;
        //      -y  x  w
        W(0, 0) = -quaternion(1);
        W(0, 1) = -quaternion(2);
        W(0, 2) = -quaternion(3);

        W(1, 0) = quaternion(0);
        W(1, 1) = -quaternion(3);
        W(1, 2) = quaternion(2);

        W(2, 0) = quaternion(3);
        W(2, 1) = quaternion(0);
        W(2, 2) = -quaternion(1);

        W(3, 0) = -quaternion(2);
        W(2, 1) = quaternion(1);
        W(2, 2) = quaternion(0);

        W = 0.5 * dt_ * W;
    }

    void orien_ekf::quat_2_Rot(VectorXd &quaternion, MatrixXd &Rotation)
    {

        Quaterniond q;
        q.w() = quaternion(0);
        q.x() = quaternion(1);
        q.y() = quaternion(2);
        q.z() = quaternion(3);
        Rotation = q.normalized().toRotationMatrix();
    }

    void orien_ekf::quat_2_H(VectorXd &quaternion, MatrixXd &H)
    {
        double w = quaternion(0);
        double x = quaternion(1);
        double y = quaternion(2);
        double z = quaternion(3);

        H(0, 0) = gravity_(0) * w + gravity_(1) * z - gravity_(2) * y;
        H(0, 1) = gravity_(0) * x + gravity_(1) * y + gravity_(2) * z;
        H(0, 2) = -gravity_(0) * y + gravity_(1) * x - gravity_(2) * w;
        H(0, 3) = -gravity_(0) * z + gravity_(1) * w + gravity_(2) * x;

        H(1, 0) = -gravity_(0) * z + gravity_(1) * w + gravity_(2) * x;
        H(1, 1) = gravity_(0) * y - gravity_(1) * x + gravity_(2) * w;
        H(1, 2) = gravity_(0) * x + gravity_(1) * y + gravity_(2) * z;
        H(1, 3) = -gravity_(0) * w - gravity_(1) * z + gravity_(2) * y;

        H(2, 0) = gravity_(0) * y - gravity_(1) * x + gravity_(2) * w;
        H(2, 1) = gravity_(0) * z - gravity_(1) * w - gravity_(2) * x;
        H(2, 2) = gravity_(0) * w + gravity_(1) * z - gravity_(2) * y;
        H(2, 3) = gravity_(0) * x + gravity_(1) * y + gravity_(2) * z;
        H = 2 * H;
    }

    void orien_ekf::quaternionToEuler(VectorXd quaternion, double &yaw, double &pitch, double &roll)
    {
        double qw = quaternion(0);
        double qx = quaternion(1);
        double qy = quaternion(2);
        double qz = quaternion(3);
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

    void orien_ekf::quat_norm(VectorXd &quaternion)
    {
        double norm = quaternion.norm();
        quaternion = quaternion / norm;
    }

    void orien_ekf::get_measurement()
    {
        angular_vel_stack_.push_back(angular_vel_b_);
        accel_stack_.push_back(accel_b_);
        imu_time_stack_.push_back(imu_time_);
        discrete_time_stack_.push_back(discrete_time);
        filter_quaternion_stack_.push_back(quaternion_);
        Cov_stack_.push_back(Cov_q_);

        if (vo_new_ && imu_time_stack_.size() > 0) // log when new vo transformation was subscribed
        {
            auto start1 = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count();

            vo_new_ = false; // meassage logged
            double _vo_time = vo_time_;
            VectorXd vo_reading = vo_pose_quaternion_;

            // sychronize vo timestamp to the left first imu timestamp
            // imu time is the rclcpp::clock.now() when the imu_callback gets callled
            auto idx_pre_ptr = std::upper_bound(imu_time_stack_.begin(),
                                                imu_time_stack_.end(), _vo_time); // find the first imu time bigger than vo_time_pre_

            if (idx_pre_ptr == imu_time_stack_.begin())
            {
                std::cout << "not storing enough imu info, failed to correct" << std::endl;
                // should happens at the beginning of the MHE
                // discard the vo_meas
            }
            else // this if condition should only happens at during the initalization, so give some time for the imu to store enough info
            {
                int imu_sychron_idx = std::distance(imu_time_stack_.begin(), idx_pre_ptr) - 1;
                int relative_discrete_time = discrete_time_stack_.back() - discrete_time_stack_[imu_sychron_idx];
                quaternion_ = filter_quaternion_stack_[imu_sychron_idx];
                Cov_q_ = Cov_stack_[imu_sychron_idx];

                for (int i = 0; i < relative_discrete_time - 1; i++)
                {
                    gyro_nonlinear_predict(quaternion_pred_, quaternion_, angular_vel_stack_[imu_sychron_idx + i], Cov_q_, Cov_q_pred_);
                    gyro_nonlinear_correct(quaternion_correct_, quaternion_pred_, accel_stack_[imu_sychron_idx + i], Cov_q_pred_, Cov_q_correct_);

                    if (i == 0)
                    {
                        quaternion_pred_ = quaternion_correct_;
                        Cov_q_pred_ = Cov_q_correct_;
                        vo_nonlinear_correct(quaternion_correct_, quaternion_pred_, vo_reading, Cov_q_pred_, Cov_q_correct_);
                    }

                    quaternion_ = quaternion_correct_;
                    Cov_q_ = Cov_q_correct_;
                }
            }

            auto stop1 = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count();
            auto duration1 = static_cast<double>(stop1 - start1) / 1000000;
            // std::cout << "Total loop hz elapsed: " << 1 / duration1 << " hz." << std::endl;
            std::cout << 1 / duration1 << std::endl;
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<orien_ekf::orien_ekf>("orien_sub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}