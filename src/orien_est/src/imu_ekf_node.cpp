#include "imu_ekf.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<imu_ekf::imu_ekf>("imu_ekf");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}