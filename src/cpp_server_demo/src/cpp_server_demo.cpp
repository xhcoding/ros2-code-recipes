#include "rclcpp/executors.hpp"
#include "demo_interfaces/srv/add_int.hpp"

void AddInt(demo_interfaces::srv::AddInt_Request::SharedPtr request,
            demo_interfaces::srv::AddInt_Response::SharedPtr response) {
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("add_int_server");
    auto service = node->create_service<demo_interfaces::srv::AddInt>("add_int", AddInt);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add ints.");

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
