#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"

int main(int argc, char* argv[]) {
    
    // 初始化
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("cpp_node");
    
    RCLCPP_INFO(node->get_logger(), "Hello, C++ Node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();

    return 0;
}
