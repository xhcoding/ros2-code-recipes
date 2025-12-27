#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"
#include "demo_interfaces/msg/demo.hpp"

using namespace std::chrono_literals;

class TopicSubscribeDemo : public rclcpp::Node {
public:
    TopicSubscribeDemo(): Node("topic_subscribe_demo_node") {
        auto topic_callback = [this](demo_interfaces::msg::Demo::UniquePtr msg) {
            RCLCPP_INFO(get_logger(), "I received: %s", msg->name.data.c_str());
        };
        subscriber_ = create_subscription<demo_interfaces::msg::Demo>("topic_string", 10, topic_callback);
    }

private:
    rclcpp::Subscription<demo_interfaces::msg::Demo>::SharedPtr subscriber_;
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<TopicSubscribeDemo>());

    rclcpp::shutdown();
    return 0;
}
