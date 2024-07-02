# Go1_example package
This package deploys the Decentralized MHE & EKF estimation on Unitree go1 robot. 
## Package structure
Creating a subclass of the "decentral_legged_est/EstSub.hpp" and configuring callbacks to Receive: IMU, Encoder, and Kinematics Measurements;
1. config/parameters_go1.yaml: params config for decentral_legged_est, orien_est and orbslam3_ros2;
2. launch/go1_launch.py: launch file for decentral_legged_est, orien_est and orbslam3_ros2;
3. est_sub node: subs to IMU, Encoder, vo_relative_translation, orientation_filter and mocap_gt topics, estimates the floating base position and velocity at 200hz.
4. orien_sub node: subs to IMU and vo_pose topics, estimates the floating base orientation at 500hz/IMU frequency.
5. vo_sub: node: subs to stereo_image topics, providing ros2 interface of orbslam3 at 30hz/image frequency.