#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"
#include "demo_interfaces/msg/demo.hpp"

using namespace std::chrono_literals;

class TopicPublishDemo : public rclcpp::Node {
public:
    TopicPublishDemo(): Node("topic_publish_demo_node"), count_(0) {
        publisher_ = create_publisher<demo_interfaces::msg::Demo>("topic_string", 10);
        auto timer_callback = [this]() {
            auto msg = demo_interfaces::msg::Demo();
            msg.name.data = "Hello, Now is " + std::to_string(count_++);
            RCLCPP_INFO(get_logger(), "Publishing: %s", msg.name.data.c_str());
            publisher_->publish(msg);
        };
        timer_ = create_wall_timer(1s, timer_callback);

    }


private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<demo_interfaces::msg::Demo>::SharedPtr publisher_;
    size_t count_;
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<TopicPublishDemo>());

    rclcpp::shutdown();
    return 0;
}
