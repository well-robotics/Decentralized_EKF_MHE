# Decentralized_EKF_MHE
## Fast Decentralized State Estimation for Legged Robot Locomotion via EKF and MHE

Decentralized_EKF_MHE decentralized the nonlinear estimation of the floating base states to an orientation estimation via Extended Kalman Filter (EKF) and a linear velocity estimation via Moving Horizon Estimation (MHE). The EKF fuses the inertia sensor with vision to estimate the floating base orientation. The MHE uses the estimated orientation with all the sensors within a time window in the past to estimate the linear velocities based on a time-varying linear dynamics formulation of the interested states with state constraints. This decoupling of state estimation promotes the desired balance of computation efficiency, accuracy of estimation, and the inclusion of state constraints. The proposed method is shown to be capable of providing accurate state estimation to several legged robots, including the highly dynamic hopping robot PogoX, the bipedal robot Cassie, and the quadrupedal robot Unitree Go1, with a frequency at 200 Hz and a window interval of 0.1s.

For more details, please refer to the paper.[arxiv](http://www.arxiv.org/abs/2405.20567) 