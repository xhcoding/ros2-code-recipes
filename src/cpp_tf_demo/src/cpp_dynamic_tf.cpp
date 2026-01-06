#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/executors.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

using namespace std::chrono_literals;

class DynamicTf : public rclcpp::Node {

public:
    DynamicTf() : Node("DynamicTf"), count_(0) {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = create_wall_timer(1s, [this]{
            // publish
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = get_clock()->now();  // 时间戳
            transform.header.frame_id = "base_laser";      // 父坐标系名称
            transform.child_frame_id = "wall_point";      // 子坐标系名称
            transform.transform.translation.x = 60 + count_ * 10; // 每秒增加 10 cn
            transform.transform.translation.y = 0;
            transform.transform.translation.z = 0;

            // 旋转角度为 0
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            transform.transform.rotation = tf2::toMsg(q);

            // 发布消息
            broadcaster_->sendTransform(transform);
            
            count_++;
        });

    }

private:
    int count_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<DynamicTf>());

    rclcpp::shutdown();

    return 0;
}
