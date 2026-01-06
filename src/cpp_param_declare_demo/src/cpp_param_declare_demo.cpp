#include "rclcpp/executors.hpp"

class ParamNode : public rclcpp::Node {
public:
    ParamNode() : Node("param_node") {
        // 声明参数
        declare_parameter("int_param", 10086);
        // 获取参数默认值
        get_parameter("int_param", int_param_);
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "int param default value is : %ld", int_param_);
        
        auto paramCb = [this](const std::vector<rclcpp::Parameter>& parameters) -> auto {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& parameter : parameters) {
                int_param_ = parameter.as_int();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "param changed to : %ld", int_param_);
            }
            return result;
        };
        handle_ = add_on_set_parameters_callback(paramCb);
    }

private:
    int64_t int_param_;
    OnSetParametersCallbackHandle::SharedPtr handle_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ParamNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
