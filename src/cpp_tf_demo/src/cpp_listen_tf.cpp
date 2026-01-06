#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/executors.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.hpp"

using namespace std::chrono_literals;

class ListenTf : public rclcpp::Node {

public:
    ListenTf() : Node("ListenTf") {
        buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
        
        timer_ = create_wall_timer(1s, std::bind(&ListenTf::listen, this));
    }
    
    void listen() {
        try {
            // 查询坐标关系
            const auto transform = buffer_->lookupTransform("base_link", "wall_point", tf2::TimePointZero);
            auto translation = transform.transform.translation;
            RCLCPP_INFO(this->get_logger(), "transform: (%f, %f, %f)" , translation.x, translation.y, translation.z);
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s" , e.what());
        }
        
    }

private:
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> buffer_;
};

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ListenTf>());

    rclcpp::shutdown();

    return 0;
}
