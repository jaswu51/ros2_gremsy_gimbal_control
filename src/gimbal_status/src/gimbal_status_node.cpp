#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "payloadSdkInterface.h"

// 定义连接参数
T_ConnInfo s_conn = {
    CONTROL_UDP,
    udp_ip_target,
    udp_port_target
};

class GimbalStatusNode : public rclcpp::Node {
public:
    GimbalStatusNode() : Node("gimbal_status_node") {
        // 创建发布者
        attitude_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("gimbal_attitude", 10);
        zoom_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("gimbal_zoom", 10);

        // 初始化SDK
        initializePayloadSDK();

        // 创建定时器，100Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&GimbalStatusNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "GimbalStatusNode initialized");
    }

    ~GimbalStatusNode() {
        if (my_payload_) {
            my_payload_->sdkQuit();
            delete my_payload_;
        }
    }

private:
    void initializePayloadSDK() {
        try {
            my_payload_ = new PayloadSdkInterface(s_conn);
            my_payload_->sdkInitConnection();
            my_payload_->checkPayloadConnection();
            
            // 注册回调函数
            my_payload_->regPayloadStatusChanged(
                std::bind(&GimbalStatusNode::onPayloadStatusChanged, this, 
                std::placeholders::_1, std::placeholders::_2));

            // 设置参数更新频率
            my_payload_->setParamRate(PARAM_EO_ZOOM_LEVEL, 100);  // 100ms
            my_payload_->setParamRate(PARAM_IR_ZOOM_LEVEL, 100);  // 100ms
            
            RCLCPP_INFO(this->get_logger(), "PayloadSDK initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize PayloadSDK: %s", e.what());
            throw;
        }
    }

    void timer_callback() {
        if (!my_payload_) return;
        // 定时器只用于保持节点活跃，实际数据通过回调函数处理
    }

    void onPayloadStatusChanged(int event, double* param) {
        switch(event) {
            case PAYLOAD_GB_ATTITUDE: {
                // param[0]: pitch
                // param[1]: roll
                // param[2]: yaw
                auto msg = geometry_msgs::msg::Pose();
                
                // 将欧拉角转换为四元数（简化版本）
                double pitch = param[0] * M_PI / 180.0;  // 转换为弧度
                double roll = param[1] * M_PI / 180.0;
                double yaw = param[2] * M_PI / 180.0;
                
                msg.orientation.x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
                msg.orientation.y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
                msg.orientation.z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
                msg.orientation.w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);

                msg.position.x = param[0];  // pitch in degrees
                msg.position.y = param[1];  // roll in degrees
                msg.position.z = param[2];  // yaw in degrees

                attitude_publisher_->publish(msg);
                RCLCPP_DEBUG(this->get_logger(), "Published attitude - Pitch: %.2f, Roll: %.2f, Yaw: %.2f",
                    param[0], param[1], param[2]);
                break;
            }
            case PAYLOAD_PARAMS: {
                // param[0]: param index
                // param[1]: value
                auto msg = geometry_msgs::msg::Pose();
                if (param[0] == PARAM_EO_ZOOM_LEVEL) {
                    msg.position.x = param[1];  // EO zoom level
                    msg.position.y = last_ir_zoom_;
                    zoom_publisher_->publish(msg);
                    last_eo_zoom_ = param[1];
                    RCLCPP_DEBUG(this->get_logger(), "EO Zoom Level: %.2f", param[1]);
                }
                else if (param[0] == PARAM_IR_ZOOM_LEVEL) {
                    msg.position.x = last_eo_zoom_;
                    msg.position.y = param[1];  // IR zoom level
                    zoom_publisher_->publish(msg);
                    last_ir_zoom_ = param[1];
                    RCLCPP_DEBUG(this->get_logger(), "IR Zoom Level: %.2f", param[1]);
                }
                break;
            }
        }
    }

    PayloadSdkInterface* my_payload_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr attitude_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr zoom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double last_eo_zoom_{0.0};
    double last_ir_zoom_{0.0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GimbalStatusNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 