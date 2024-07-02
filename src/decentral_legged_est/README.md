## Decentral_legged_est package
This package implements the Decentralized MHE estimation for legged robots.
## Package structure
1. *EstSub.hpp*: Superclass interface for hardware deployment, sub to orientation EKF and VO_ros2 interface by default.
2. *DecentralEst.hpp*: Setup the MHE problem as constrained optimization problem.
3. *MheSrb.hpp*: Configurate the MHE as Quadratic Program, solved using OSQP sparse optimization. 
