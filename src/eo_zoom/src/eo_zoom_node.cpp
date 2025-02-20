#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include "payloadSdkInterface.h"

#if (CONTROL_METHOD == CONTROL_UART)
T_ConnInfo s_conn = {
    CONTROL_UART,
    payload_uart_port,
    payload_uart_baud
};
#else
T_ConnInfo s_conn = {
    CONTROL_UDP,
    udp_ip_target,
    udp_port_target
};
#endif

class EoZoomNode : public rclcpp::Node {
private:
    PayloadSdkInterface* my_payload;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr zoom_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr zoom_range_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr focus_cmd_sub_;

public:
    EoZoomNode() : Node("eo_zoom_node") {
        RCLCPP_INFO(this->get_logger(), "启动 EO 变焦节点...");
        
        // 创建 PayloadSDK 接口
        my_payload = new PayloadSdkInterface(s_conn);
        
        // 初始化连接
        my_payload->sdkInitConnection();
        RCLCPP_INFO(this->get_logger(), "等待负载设备连接...");
        my_payload->checkPayloadConnection();

        // 设置为可见光视图
        RCLCPP_INFO(this->get_logger(), "设置为可见光视图");
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_EO, PARAM_TYPE_UINT32);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // 设置初始变焦级别
        #if defined GHADRON
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_FACTOR, ZOOM_EO_1X, PARAM_TYPE_UINT32);
        #elif defined VIO || defined ZIO
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_1X, PARAM_TYPE_UINT32);
        #endif

        // 创建订阅者
        zoom_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "eo_zoom/command", 10,
            std::bind(&EoZoomNode::zoomCommandCallback, this, std::placeholders::_1));
            
        zoom_range_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "eo_zoom/range", 10,
            std::bind(&EoZoomNode::zoomRangeCallback, this, std::placeholders::_1));

        #if defined VIO || defined ZIO
        focus_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "eo_zoom/focus", 10,
            std::bind(&EoZoomNode::focusCommandCallback, this, std::placeholders::_1));
        #endif

        RCLCPP_INFO(this->get_logger(), "EO 变焦节点已准备就绪");
    }

    ~EoZoomNode() {
        if (my_payload != nullptr) {
            try {
                my_payload->sdkQuit();
                delete my_payload;
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "关闭设备时发生错误");
            }
        }
    }

private:
    void zoomCommandCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "zoom_in") {
            RCLCPP_INFO(this->get_logger(), "执行放大");
            my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_IN);
        } 
        else if (msg->data == "zoom_out") {
            RCLCPP_INFO(this->get_logger(), "执行缩小");
            my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_OUT);
        }
        else if (msg->data == "stop") {
            RCLCPP_INFO(this->get_logger(), "停止变焦");
            my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_STOP);
        }
    }

    void zoomRangeCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        float range = msg->data;
        if (range >= 0.0 && range <= 100.0) {
            RCLCPP_INFO(this->get_logger(), "设置变焦范围: %.1f%%", range);
            my_payload->setCameraZoom(ZOOM_TYPE_RANGE, range);
        } else {
            RCLCPP_WARN(this->get_logger(), "变焦范围无效: %.1f", range);
        }
    }

    #if defined VIO || defined ZIO
    void focusCommandCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "focus_in") {
            RCLCPP_INFO(this->get_logger(), "执行调焦近");
            my_payload->setCameraFocus(FOCUS_TYPE_CONTINUOUS, FOCUS_IN);
        }
        else if (msg->data == "focus_out") {
            RCLCPP_INFO(this->get_logger(), "执行调焦远");
            my_payload->setCameraFocus(FOCUS_TYPE_CONTINUOUS, FOCUS_OUT);
        }
        else if (msg->data == "stop") {
            RCLCPP_INFO(this->get_logger(), "停止调焦");
            my_payload->setCameraFocus(FOCUS_TYPE_CONTINUOUS, FOCUS_STOP);
        }
        else if (msg->data == "auto") {
            RCLCPP_INFO(this->get_logger(), "自动对焦");
            my_payload->setCameraFocus(FOCUS_TYPE_AUTO);
        }
    }
    #endif
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EoZoomNode>());
    rclcpp::shutdown();
    return 0;
}