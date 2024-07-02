## Decentral_legged_est package
This package implements the Decentralized MHE estimation for legged robots.
## Package structure
1. "EstSub.hpp": superclass interface for hardware deployment, sub to orientation EKF and VO_ros2 interface by default.
2. "DecentralEst.hpp": setup the MHE problem as constrained optimization problem.
3. "MheSrb.hpp": configurate the MHE as Quadratic Program, solved using OSQP sparse optimization. 
