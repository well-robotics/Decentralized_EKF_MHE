## decentral_legged_est package
This package implement the Decentralized MHE estimation for legged robot.
## package structure
1. "EstSub.hpp": superclass interface for hardware deployment, sub to orientation EKF and vo ros2 interface.
2. "DecentralEst.hpp": setup the MHE problem as process constrained optimization problem.
3. "MheSrb.hpp": configurate the MHE as quadratic program, solved using OSQP sparse optiumization. 
