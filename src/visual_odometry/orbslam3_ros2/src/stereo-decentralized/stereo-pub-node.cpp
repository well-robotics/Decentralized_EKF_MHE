#include "stereo-pub-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

StereoPubNode::StereoPubNode(ORB_SLAM3::System *pSLAM, const string &strSettingsFile, const string &strDoRectify)
    : Node("ORB_SLAM3_ROS2"),
      m_SLAM(pSLAM)
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;
    std::cout << strSettingsFile << std::endl;
    if (doRectify)
    {
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);
    }

    // the pogox body to cam transformation
    //-----------------------------------------------------------
    // T_body_to_camera_.translation() = Eigen::Vector3d(0.12615755, 0.06577286, -0.14597424);
    // q_body_to_camera_ = Eigen::Quaterniond(0.5021, -0.5018, 0.4993, -0.4968); // the pogox body to cam transformation
    // T_body_to_camera_.rotate(q_body_to_camera_.matrix());

    // the go1 body (imu) to cam transformation
    //-----------------------------------------------------------
    // T_ic: (cam0 to imu0):
    // [[-0.01565831 -0.04952191 0.99865029 0.10578079]
    // [-0.99987729 0.00029446 -0.01566294 0.1104795 ]
    // [ 0.0004816 -0.99877299 -0.04952044 0.11233088]

    Eigen::Matrix3d T_body_to_camera_rotation;
    T_body_to_camera_rotation << -0.01565831, -0.04952191, 0.99865029,
        -0.99987729, 0.00029446, -0.01566294,
        0.0004816, -0.99877299, -0.04952044;
    T_body_to_camera_.translation() = Eigen::Vector3d(0.10578079, 0.1104795, 0.11233088);
    T_body_to_camera_.rotate(T_body_to_camera_rotation);

    // auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(shared_ptr<rclcpp::Node>(this), "/camera/infra1/image_rect_raw");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(shared_ptr<rclcpp::Node>(this), "/camera/infra2/image_rect_raw");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(50), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoPubNode::GrabStereo, this);

    publish_pose_stamped = create_publisher<custom_msgs::msg::VoRealtiveTransform>("orb/vo", 3);
    publish_pose = create_publisher<geometry_msgs::msg::PoseStamped>("orb/pos", 3);
}

StereoPubNode::~StereoPubNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    // m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoPubNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{

    Cv_TimeStamp = rclcpp::Clock().now();

    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    Sophus::SE3f vo_pose_SE3f;

    float imageScale = m_SLAM->GetImageScale();
    cv::Mat im, imRight;

    if (imageScale != 1.f)
    {
        int width = cv_ptrLeft->image.cols * imageScale;
        int height = cv_ptrLeft->image.rows * imageScale;
        // std::cout << width << "||" << height << std::endl;
        cv::resize(cv_ptrLeft->image, im, cv::Size(width, height));
        cv::resize(cv_ptrRight->image, imRight, cv::Size(width, height));
    }
    else
    {
        im = cv_ptrLeft->image.clone();
        imRight = cv_ptrRight->image.clone();
    }

    vo_pose_SE3f = m_SLAM->TrackStereo(im, imRight, Utility::StampToSec(msgLeft->header.stamp));

    vo_pose_SE3f = vo_pose_SE3f.inverse(); // I dont know why, but the output is the inverse of the realtive transformation from the last image to current image.

    // ----------------------------------------------------------------
    // use Eigen::Isometry3d as template to compute
    Eigen::Isometry3d T_world_to_camera = Eigen::Isometry3d::Identity();
    T_world_to_camera.translation() = Eigen::Vector3d(vo_pose_SE3f.translation().x(), vo_pose_SE3f.translation().y(), vo_pose_SE3f.translation().z());
    double w = vo_pose_SE3f.unit_quaternion().coeffs().w();
    double x = vo_pose_SE3f.unit_quaternion().coeffs().x();
    double y = vo_pose_SE3f.unit_quaternion().coeffs().y();
    double z = vo_pose_SE3f.unit_quaternion().coeffs().z();
    Eigen::Quaterniond world_to_camera_quaternion_ = Eigen::Quaterniond(w, x, y, z); // w x y z
    T_world_to_camera.rotate(world_to_camera_quaternion_.matrix());

    // ----------------------------------------------------------------
    if (init == 0)
    {
        T_world_to_body_init_ = T_world_to_camera * T_body_to_camera_.inverse();
    }

    if (init > 0)
    {
        // Compute the relative transformation from body_pre to body
        Eigen::Isometry3d realtive_body_12 = T_body_to_camera_ * T_world_to_camera_pre_.inverse() * T_world_to_camera * T_body_to_camera_.inverse();

        Eigen::Isometry3d T_world_to_body = T_world_to_body_init_.inverse() * T_world_to_camera * T_body_to_camera_.inverse();

        Eigen::Quaterniond q_world_to_body(T_world_to_body.rotation());

        // ------------------------------------------------
        geometry_msgs::msg::PoseStamped pose_world_2_body;
        pose_world_2_body.header.stamp = Cv_TimeStamp;
        pose_world_2_body.pose.position.x = T_world_to_body.translation().x();
        pose_world_2_body.pose.position.y = T_world_to_body.translation().y();
        pose_world_2_body.pose.position.z = T_world_to_body.translation().z();

        pose_world_2_body.pose.orientation.x = q_world_to_body.x();
        pose_world_2_body.pose.orientation.y = q_world_to_body.y();
        pose_world_2_body.pose.orientation.z = q_world_to_body.z();
        pose_world_2_body.pose.orientation.w = q_world_to_body.w();
        
        publish_pose->publish(pose_world_2_body);

        // ------------------------------------------------
        custom_msgs::msg::VoRealtiveTransform vo_relative_p;
        vo_relative_p.header.stamp = Cv_TimeStamp;
        vo_relative_p.header_pre.stamp = Cv_TimeStamp_pre;
        vo_relative_p.header.frame_id = "body";
        vo_relative_p.header_pre.frame_id = "body";

        vo_relative_p.x_relative = realtive_body_12.translation().x();
        vo_relative_p.y_relative = realtive_body_12.translation().y();
        vo_relative_p.z_relative = realtive_body_12.translation().z();

        publish_pose_stamped->publish(vo_relative_p);
    }

    T_world_to_camera_pre_ = T_world_to_camera;
    Cv_TimeStamp_pre = Cv_TimeStamp;
    init += 1;
}
