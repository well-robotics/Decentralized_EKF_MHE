# Go1_example package
This package deploys the Decentralized MHE & EKF estimation on Unitree go1 robot. 
## Package structure
Creating a subclass of the **decentral_legged_est/EstSub.hpp** and configuring callbacks to receive: *IMU*, *Encoder*, and *Kinematics* measurements;
1. **config/parameters_go1.yaml**: Params configuration for decentral_legged_est, orien_est and orbslam3_ros2;
2. **launch/go1_launch.py**: Launch file for decentral_legged_est, orien_est and orbslam3_ros2;
3. **est_sub node**: Subscribes to IMU, Encoder, vo_relative_translation, orientation_filter and mocap_gt topics, estimates the floating base position and velocity at 200hz.
4. **orien_sub node**: Subscribes to IMU and vo_pose topics, estimates the floating base orientation at 500hz/IMU frequency.
5. **vo_sub: node**: Subscribes to stereo_image topics, providing ros2 interface of orbslam3 at 30hz/image frequency.