#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include "payloadSdkInterface.h"

// ... existing connection setup code from ir_set_camera_zoom.cpp ...

class IrZoomNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber zoom_cmd_sub_;
    ros::Subscriber zoom_range_sub_;
    PayloadSdkInterface* my_payload;

public:
    IrZoomNode() {
        // 创建 PayloadSDK 接口
        my_payload = new PayloadSdkInterface(s_conn);
        
        // 初始化连接
        my_payload->sdkInitConnection();
        ROS_INFO("等待负载设备连接...");
        my_payload->checkPayloadConnection();

        // 设置为红外视图
        ROS_INFO("设置为红外视图");
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_IR, PARAM_TYPE_UINT32);
        ros::Duration(1.0).sleep();

        // 设置初始变焦级别
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_ZOOM_FACTOR, ZOOM_IR_1X, PARAM_TYPE_UINT32);
        
        // 订阅变焦命令话题
        zoom_cmd_sub_ = nh_.subscribe("ir_zoom/command", 1, &IrZoomNode::zoomCommandCallback, this);
        zoom_range_sub_ = nh_.subscribe("ir_zoom/range", 1, &IrZoomNode::zoomRangeCallback, this);
    }

    ~IrZoomNode() {
        if (my_payload != nullptr) {
            my_payload->sdkQuit();
            delete my_payload;
        }
    }

    void zoomCommandCallback(const std_msgs::String::ConstPtr& msg) {
        if (msg->data == "zoom_in") {
            my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_IN);
        } 
        else if (msg->data == "zoom_out") {
            my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_OUT);
        }
        else if (msg->data == "stop") {
            my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_STOP);
        }
    }

    void zoomRangeCallback(const std_msgs::Float32::ConstPtr& msg) {
        float range = msg->data;
        if (range >= 0.0 && range <= 100.0) {
            my_payload->setCameraZoom(ZOOM_TYPE_RANGE, range);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ir_zoom_node");
    
    IrZoomNode zoom_node;
    
    ros::spin();
    
    return 0;
}