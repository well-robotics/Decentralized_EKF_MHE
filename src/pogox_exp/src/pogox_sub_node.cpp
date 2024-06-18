#include <pogox_sub.hpp>
#include <iostream>

int main(int argc, char **argv)
{

    // if (__cplusplus == 202101L) std::cout << "C++23";
    // else if (__cplusplus == 202002L) std::cout << "C++20";
    // else if (__cplusplus == 201703L) std::cout << "C++17";
    // else if (__cplusplus == 201402L) std::cout << "C++14";
    // else if (__cplusplus == 201103L) std::cout << "C++11";
    // else if (__cplusplus == 199711L) std::cout << "C++98";
    // else std::cout << "pre-standard C++." << __cplusplus;
    // std::cout << "\n";

    rclcpp::init(argc, argv);
    auto node = std::make_shared<pogoxSub::pogoxSub>("pogox_Sub");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
