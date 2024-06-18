#include <EstSub.hpp>
#include <iostream>

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<robotSub::robotSub>("est_sub");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
