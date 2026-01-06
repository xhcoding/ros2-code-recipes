#include "rclcpp/executors.hpp"
#include "demo_interfaces/srv/add_int.hpp"

using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("add_client");
    auto client = node->create_client<demo_interfaces::srv::AddInt>("add_int");
    auto request = std::make_shared<demo_interfaces::srv::AddInt_Request>();
    request->a = 100;
    request->b = 200;
    
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
    }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }
    rclcpp::shutdown();
    return 0;
}
