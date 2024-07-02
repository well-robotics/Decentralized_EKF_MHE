# go1_example package
This package deploys the Decentralized MHE + EKF estimation on Unitree go1 robot. 
## package structure
Creating a Subclass of the "decentral_legged_est/EstSub.hpp" and configuring callbacks to Receive IMU, Encoder, and Kinematics Measurements
1. config/parameters_go1.yaml: params config for decentral_legged_est, orien_est, orbslam3;
2. launch/go1_launch.py: launch for decentral_legged_est, orien_est, orbslam3;
3. est_sub node subs to IMU, Encoder, vo_relative_translation and mocap_gt topics, estimates the floating base position and velocity at 200hz.
4. orien_sub node subs to IMU, vo_pose topics, estimates the floating base orientation at 500hz/IMU frequency.
5. vo_sub node subs to stereo_image topics, providing ros interface of orbslam3 at 30hz/Image frequency.