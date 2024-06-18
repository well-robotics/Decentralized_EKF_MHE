#include "EstSub.hpp"

using namespace Eigen;
using namespace std;

namespace EstSub
{

    EstSub::EstSub(const std::string &name) : Node(name)
    {
        robot_store_ = std::make_shared<robot_store>();
        robot_params_ = std::make_shared<robot_params>();

        paramsWrapper();

        est_type_ = robot_params_->est_type_;

        int timer_interval_ = this->get_parameter("interval").as_int();
        string data_path = this->get_parameter("data_path").as_string();

        infile_meas_.open(data_path + "/imu_kinematic_measurements.txt");
        infile_vel_gt_.open(data_path + "/gt_velocity.txt");
        infile_orien_gt_.open(data_path + "/gt_orientation.txt");
        infile_vel_iekf_.open(data_path + "/iekf_vel.txt");
        infile_orien_iekf_.open(data_path + "/iekf_orien.txt");

        timer_ = create_wall_timer(std::chrono::milliseconds(timer_interval_), std::bind(&EstSub::timerCallback, this));

        InitOrienEKF();
    }

    EstSub::~EstSub()
    {
    }

    void EstSub::InitOrienEKF()
    {
        gravity_ << 0, 0, 9.81;
        C_gyro_.diagonal() << std::pow(0.01, 2), std::pow(0.01, 2), std::pow(0.01, 2);
        C_accel_.diagonal() << std::pow(8.5, 2), std::pow(8.5, 2), std::pow(8.5, 2);

        quternion_prior_ = VectorXd::Zero(4);
        quternion_prior_(0) = 1;
        quaternion_ = quternion_prior_;
        Cov_q_ = 0.001 * 0.001 * MatrixXd::Identity(4, 4);
    }

    void EstSub::timerCallback()
    {
        auto start = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count();

        //---------------------------------------------------------------
        // Loop through measurement data file and read in measurements line by line
        string line_meas;
        Eigen::Matrix<double, 6, 1> imu_measurement = Eigen::Matrix<double, 6, 1>::Zero();

        for (int m = 0; m < 3; m++)
        {
            getline(infile_meas_, line_meas);

            vector<string> measurement;
            boost::split(measurement, line_meas, boost::is_any_of(" "));
            // read 3 next lines, which idealy should be IMU, contact and kinematics
            // // Handle measurements
            if (measurement[0].compare("IMU") == 0)
            {
                // cout << "Received IMU Data, propagating state\n";
                assert((measurement.size() - 2) == 6);
                t_ = atof(measurement[1].c_str());
                // Read in IMU data
                imu_measurement << stod98(measurement[2]),
                    stod98(measurement[3]),
                    stod98(measurement[4]),
                    stod98(measurement[5]),
                    stod98(measurement[6]),
                    stod98(measurement[7]);

                // Propagate using IMU data
                double dt = t_ - t_prev_;
                robot_store_->delta_t_ = dt;
                robot_store_->accel_b_(0) = imu_measurement(3, 0);
                robot_store_->accel_b_(1) = imu_measurement(4, 0);
                robot_store_->accel_b_(2) = imu_measurement(5, 0);

                robot_store_->angular_b_(0) = imu_measurement(0, 0);
                robot_store_->angular_b_(1) = imu_measurement(1, 0);
                robot_store_->angular_b_(2) = imu_measurement(2, 0);
            }
            else if (measurement[0].compare("CONTACT") == 0)
            {
                // cout << "Received CONTACT Data, setting filter's contact state\n";
                assert((measurement.size() - 2) % 2 == 0);
                vector<pair<int, bool>> contacts;
                int id;
                bool indicator;
                t_ = stod98(measurement[1]);
                // Read in contact data
                for (int i = 2; i < measurement.size(); i += 2)
                {
                    id = stoi98(measurement[i]);
                    indicator = bool(stod98(measurement[i + 1]));

                    if (id == 0.0)
                    {
                        robot_store_->contact_left_ = indicator;
                    }
                    else if (id == 1.0)
                    {
                        robot_store_->contact_right_ = indicator;
                    }
                }
            }
            else if (measurement[0].compare("KINEMATIC") == 0)
            {
                // cout << "Received KINEMATIC observation, correcting state\n";
                assert((measurement.size() - 2) % 44 == 0);
                int id;
                Eigen::Quaternion<double> q;
                Eigen::Vector3d p;
                Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
                Eigen::Matrix<double, 6, 6> covariance;
                t_ = stod98(measurement[1]);
                // Read in kinematic data
                for (int i = 2; i < measurement.size(); i += 44)
                {
                    id = stoi98(measurement[i]);
                    q = Eigen::Quaternion<double>(stod98(measurement[i + 1]), stod98(measurement[i + 2]), stod98(measurement[i + 3]), stod98(measurement[i + 4]));
                    q.normalize();
                    p << stod98(measurement[i + 5]), stod98(measurement[i + 6]), stod98(measurement[i + 7]);
                    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
                    pose.block<3, 1>(0, 3) = p;
                    for (int j = 0; j < 6; ++j)
                    {
                        for (int k = 0; k < 6; ++k)
                        {
                            covariance(j, k) = stod98(measurement[i + 8 + j * 6 + k]);
                        }
                    }

                    if (id == 0.0)
                    {
                        robot_store_->Kinematics_left_ = pose;
                        robot_store_->Cov_left_ = covariance;
                    }
                    else if (id == 1.0)
                    {
                        robot_store_->Kinematics_right_ = pose;
                        robot_store_->Cov_right_ = covariance;
                    }
                }
            }
        }
        t_prev_ = t_;

        //---------------------------------------------------------------
        // Loop through gt & iekf data file and read in gt & iekf line by line
        string line_orien_gt;
        getline(infile_orien_gt_, line_orien_gt);
        vector<string> orien_gt;
        boost::split(orien_gt, line_orien_gt, boost::is_any_of(" "));
        if (orien_gt[0].compare("ORIENTATION") == 0)
        {
            assert((orien_gt.size() - 2) % 9 == 0);
            // read in gt orientation as rotation matrix
            for (int j = 0; j < 3; ++j)
            {
                for (int k = 0; k < 3; ++k)
                {
                    Rot_gt_(j, k) = stod98(orien_gt[2 + j * 3 + k]);
                }
            }
        }
        Eigen::Quaterniond quaternion_gt(Rot_gt_);

        string line_orien_iekf;
        getline(infile_orien_iekf_, line_orien_iekf);
        vector<string> orien_iekf_string;
        boost::split(orien_iekf_string, line_orien_iekf, boost::is_any_of(" "));
        if (orien_iekf_string[0].compare("ORIENTATION") == 0)
        {
            assert((orien_iekf_string.size() - 2) % 9 == 0);
            for (int j = 0; j < 3; ++j)
            {
                for (int k = 0; k < 3; ++k)
                {
                    Rot_iekf_(j, k) = stod98(orien_iekf_string[2 + j * 3 + k]);
                }
            }
        }
        Eigen::Quaterniond quaternion_iefk(Rot_iekf_);

        string line_vel_gt;
        getline(infile_vel_gt_, line_vel_gt);
        vector<string> vel_gt_string;
        boost::split(vel_gt_string, line_vel_gt, boost::is_any_of(" "));
        if (vel_gt_string[0].compare("VELOCITY") == 0)
        {
            assert((vel_gt_string.size() - 2) % 4 == 0);
            vel_gt_ << stod98(vel_gt_string[2]), stod98(vel_gt_string[3]), stod98(vel_gt_string[4]);
        }

        string line_vel_iekf;
        getline(infile_vel_iekf_, line_vel_iekf);
        vector<string> vel_iekf_string;
        boost::split(vel_iekf_string, line_vel_iekf, boost::is_any_of(" "));
        if (vel_iekf_string[0].compare("VELOCITY") == 0)
        {
            assert((vel_iekf_string.size() - 2) % 3 == 0);
            vel_iekf_ << stod98(vel_iekf_string[2]), stod98(vel_iekf_string[3]), stod98(vel_iekf_string[4]);
        }

        // ------------------------------------------------------------------------------
        // orientation EKF
        if (discrete_time_ == 0)
        {
            quaternion_(0) = quaternion_gt.w();
            quaternion_(1) = quaternion_gt.x();
            quaternion_(2) = quaternion_gt.y();
            quaternion_(3) = quaternion_gt.z();

            gyro_nonlinear_predict(quaternion_pred_, quaternion_, robot_store_->angular_b_, Cov_q_, Cov_q_pred_, robot_store_->delta_t_);
            gyro_nonlinear_correct(quaternion_correct_, quaternion_pred_, robot_store_->accel_b_, Cov_q_pred_, Cov_q_correct_);
            quaternion_ = quaternion_correct_;

            Cov_q_ = Cov_q_correct_;
        }
        else
        {
            gyro_nonlinear_predict(quaternion_pred_, quaternion_, robot_store_->angular_b_, Cov_q_, Cov_q_pred_, robot_store_->delta_t_);
            gyro_nonlinear_correct(quaternion_correct_, quaternion_pred_, robot_store_->accel_b_, Cov_q_pred_, Cov_q_correct_);
            quaternion_ = quaternion_correct_;
            Cov_q_ = Cov_q_correct_;
        }

        MatrixXd Rotation_ekf = MatrixXd::Zero(3, 3);
        quat_2_Rot(quaternion_, Rotation_ekf);
        robot_store_->Rotation_ = Rotation_ekf;

        // ------------------------------------------------------------------------------
        // format logging
        quaternion_gt_[0] = quaternion_gt.w();
        quaternion_gt_[1] = quaternion_gt.x();
        quaternion_gt_[2] = quaternion_gt.y();
        quaternion_gt_[3] = quaternion_gt.z();

        quaternion_iekf_[0] = quaternion_iefk.w();
        quaternion_iekf_[1] = quaternion_iefk.x();
        quaternion_iekf_[2] = quaternion_iefk.y();
        quaternion_iekf_[3] = quaternion_iefk.z();

        double gt_roll, gt_pitch, gt_yaw = 0.0;
        double iekf_roll, iekf_pitch, iekf_yaw = 0.0;
        double filter_roll, filter_pitch, filter_yaw = 0.0;

        quaternionToEuler(quaternion_, filter_yaw, filter_pitch, filter_roll);
        quaternionToEuler(quaternion_iekf_, iekf_yaw, iekf_pitch, iekf_roll);
        quaternionToEuler(quaternion_gt_, gt_yaw, gt_pitch, gt_roll);

        gt_euler_[0] = gt_roll;
        gt_euler_[1] = gt_pitch;
        gt_euler_[2] = gt_yaw;

        iekf_euler_[0] = iekf_roll;
        iekf_euler_[1] = iekf_pitch;
        iekf_euler_[2] = iekf_yaw;

        filter_euler_[0] = filter_roll;
        filter_euler_[1] = filter_pitch;
        filter_euler_[2] = filter_yaw;

        // ------------------------------------------------------------------------------
        // estimator update
        if (discrete_time_ == 0)
        {
            estimator_.initialize(robot_store_, robot_params_);
            init_logging();
        }
        else
        {
            estimator_.update(discrete_time_);
            logger_.spin_logging();
        }
        discrete_time_++;

        auto stop = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count();
        auto duration = static_cast<double>(stop - start) / 1000000;
        std::cout << 1 / duration << std::endl;
    }

    void EstSub::gyro_nonlinear_predict(VectorXd &quaternion_pred, VectorXd &quaternion, Vector3d &gyro_reading, MatrixXd &Cov_q, MatrixXd &Cov_q_pred, double dt_)
    {
        MatrixXd Ohm = MatrixXd::Zero(4, 4);
        gyro_2_Ohm(gyro_reading, Ohm);

        MatrixXd W = MatrixXd::Zero(4, 3);
        quat_2_W(quaternion, W, dt_);

        MatrixXd F = MatrixXd::Identity(4, 4) + dt_ / 2 * Ohm;

        quaternion_pred = F * quaternion;

        Cov_q_pred = F * Cov_q * F.transpose() + W * C_gyro_ * W.transpose();

        quat_norm(quaternion_pred);
    }

    void EstSub::gyro_nonlinear_correct(VectorXd &quaternion_correct, VectorXd &quaternion_pred, Vector3d &accel_readings, MatrixXd &Cov_q_pred, MatrixXd &Cov_q_correct)
    {
        MatrixXd Rotation_pred = MatrixXd::Zero(3, 3);
        quat_2_Rot(quaternion_pred, Rotation_pred);

        Vector3d accel_hat = Rotation_pred.transpose() * gravity_;

        MatrixXd H = MatrixXd::Zero(3, 4);
        quat_2_H(quaternion_pred, H);

        double accel_relative_norm = accel_readings.norm() / gravity_.norm();

        MatrixXd K = Cov_q_pred * H.transpose() * (H * Cov_q_pred * H.transpose() + accel_relative_norm * accel_relative_norm * C_accel_).inverse();
        quaternion_correct = quaternion_pred + K * (accel_readings - accel_hat);
        Cov_q_correct = (MatrixXd::Identity(4, 4) - K * H) * Cov_q_pred;

        quat_norm(quaternion_correct);
    }

    void EstSub::gyro_2_Ohm(Vector3d &gyro_reading, MatrixXd &Ohm)
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
    void EstSub::quat_2_W(VectorXd &quaternion, MatrixXd &W, double &dt_)
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

    void EstSub::quat_2_Rot(VectorXd &quaternion, MatrixXd &Rotation)
    {

        Quaterniond q;
        q.w() = quaternion(0);
        q.x() = quaternion(1);
        q.y() = quaternion(2);
        q.z() = quaternion(3);
        Rotation = q.normalized().toRotationMatrix();
    }

    void EstSub::quat_2_H(VectorXd &quaternion, MatrixXd &H)
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
    void EstSub::quat_norm(VectorXd &quaternion)
    {
        double norm = quaternion.norm();
        quaternion = quaternion / norm;
    }

    // ------------------------------------------------------------------------------

    void EstSub::init_logging()
    {
        std::string log_name = this->get_parameter("log_name").as_string();

        if (est_type_ == 0)
        {
            logger_.init(log_name);
            logger_.add_data(vel_gt_, "GT_v");
            logger_.add_data(estimator_.x_MHE_, "X_MHE_");
            logger_.add_data(vel_iekf_, "vel_iekf_");
            logger_.add_data(iekf_euler_, "iekf_euler_");
            logger_.add_data(filter_euler_, "filter_euler_");
            logger_.add_data(gt_euler_, "gt_euler_");
        }
        else if (est_type_ == 1)
        {
            logger_.init(log_name);
            logger_.add_data(vel_gt_, "GT_v");
            logger_.add_data(estimator_.x_KF_, "x_KF_");
            logger_.add_data(vel_iekf_, "vel_iekf_");
            logger_.add_data(iekf_euler_, "iekf_euler_");
            logger_.add_data(filter_euler_, "filter_euler_");
            logger_.add_data(gt_euler_, "gt_euler_");
        }
    }

    void EstSub::quaternionToEuler(VectorXd &quaternion, double &yaw, double &pitch, double &roll)
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

    void EstSub::paramsWrapper()
    {
        this->declare_parameter<std::string>("data_path", "/home/data");
        this->declare_parameter<std::string>("log_name", "exp");

        // measurement params
        this->declare_parameter("p_process_std", std::vector<double>{0.01, 0.01, 0.01});
        this->declare_parameter("accel_bias_process_std", std::vector<double>{1., 1., 0.1});
        this->declare_parameter("accel_input_std", std::vector<double>{0.01, 0.04, 0.001});

        robot_params_->p_process_std_ = this->get_parameter("p_process_std").as_double_array();
        robot_params_->accel_bias_std_ = this->get_parameter("accel_bias_process_std").as_double_array();
        robot_params_->accel_input_std_ = this->get_parameter("accel_input_std").as_double_array();

        this->declare_parameter("foot_slide_std", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("foot_swing_std", 1000000000.0);

        robot_params_->foothold_slide_std_ = this->get_parameter("foot_slide_std").as_double_array();
        robot_params_->foot_swing_std_ = this->get_parameter("foot_swing_std").as_double();

        this->declare_parameter("p_init_std", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("v_init_std", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("foot_init_std", 0.025);
        this->declare_parameter("accel_bias_init_std", 0.1);

        robot_params_->p_init_std_ = this->get_parameter("p_init_std").as_double_array();
        robot_params_->v_init_std_ = this->get_parameter("v_init_std").as_double_array();
        robot_params_->foot_init_std_ = this->get_parameter("foot_init_std").as_double();
        robot_params_->accel_bias_init_std_ = this->get_parameter("accel_bias_init_std").as_double();

        // estimator params
        this->declare_parameter("rate", 50);
        this->declare_parameter("interval", 20);
        this->declare_parameter("N", 50);
        this->declare_parameter("est_type", 0);

        robot_params_->rate_ = this->get_parameter("rate").as_int();
        robot_params_->N_ = this->get_parameter("N").as_int();
        robot_params_->est_type_ = this->get_parameter("est_type").as_int();

        // OSQP params
        this->declare_parameter("rho", 0.1);
        this->declare_parameter("alpha", 1.6);
        this->declare_parameter("primTol", 0.000001);
        this->declare_parameter("dualTol", 0.000001);
        this->declare_parameter("delta", 0.00001);
        this->declare_parameter("sigma", 0.00001);
        this->declare_parameter("verbose", true);
        this->declare_parameter("adaptRho", true);
        this->declare_parameter("polish", true);
        this->declare_parameter("maxQPIter", 1000);
        this->declare_parameter("realtiveTol", 1e-3);
        this->declare_parameter("absTol", 1e-3);
        this->declare_parameter("timeLimit", 0.005);

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
        robot_params_->timeLimit_ = this->get_parameter("timeLimit").as_double();
    }

    double EstSub::stod98(const std::string &s)
    {
        return atof(s.c_str());
    }

    int EstSub::stoi98(const std::string &s)
    {
        return atoi(s.c_str());
    }

}