#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/executors.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

class StaticTf : public rclcpp::Node {

public:
    StaticTf() : Node("StaticTf") {
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // publish
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = get_clock()->now();  // 时间戳
        transform.header.frame_id = "base_link";      // 父坐标系名称
        transform.child_frame_id = "base_laser";      // 子坐标系名称
        // 坐标 (10, 0, 10)
        transform.transform.translation.x = 10;
        transform.transform.translation.y = 0;
        transform.transform.translation.z = 10;

        // 旋转角度为 0
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.transform.rotation = tf2::toMsg(q);

        // 发布消息
        broadcaster_->sendTransform(transform);
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<StaticTf>());

    rclcpp::shutdown();

    return 0;
}
