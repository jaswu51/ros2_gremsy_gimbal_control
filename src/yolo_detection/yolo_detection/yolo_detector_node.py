#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # 创建图像发布者
        self.detection_pub = self.create_publisher(
            Image, 
            'detection_box', 
            10
        )
        
        # 创建中心点发布者
        self.center_pub = self.create_publisher(
            Point,
            'detection_box_center',
            10
        )
        
        # 创建图像订阅者
        self.image_sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10
        )
        
        # 初始化 CV bridge
        self.bridge = CvBridge()
        
        # 加载 YOLO 模型
        self.get_logger().info('Loading YOLO model...')
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO model loaded successfully')

    def image_callback(self, msg):
        try:
            # 将 ROS 图像消息转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 运行目标检测
            results = self.model(cv_image)
            
            # 在图像上绘制检测结果
            annotated_frame = results[0].plot()
            
            # 获取边界框坐标
            if len(results[0].boxes) > 0:
                box = results[0].boxes[0].xyxy[0].cpu().numpy()
                
                # 计算中心点
                center_x = (box[0] + box[2]) / 2
                center_y = (box[1] + box[3]) / 2
                
                # 创建并发布中心点消息
                center_msg = Point()
                center_msg.x = float(center_x)
                center_msg.y = float(center_y)
                center_msg.z = 0.0  # 如果是2D图像，z设为0
                
                self.center_pub.publish(center_msg)
                self.get_logger().info(f'Published center point: ({center_x:.2f}, {center_y:.2f})')
                
                # 在图像上标注边界框和中心点
                box = box.astype(int)  # 转换为整数坐标
                
                # 绘制边界框
                cv2.rectangle(annotated_frame, 
                            (box[0], box[1]), 
                            (box[2], box[3]), 
                            (0, 255, 0), 2)
                
                # 绘制中心点
                cv2.circle(annotated_frame, 
                          (int(center_x), int(center_y)), 
                          5, (0, 255, 0), -1)
                
                # 添加坐标文本
                text = f"Center: ({center_x:.1f}, {center_y:.1f})"
                cv2.putText(annotated_frame, 
                          text,
                          (box[0], box[1] - 10),  # 文本位置在边界框上方
                          cv2.FONT_HERSHEY_SIMPLEX,
                          0.5,  # 字体大小
                          (0, 255, 0),  # 颜色
                          2)  # 线宽
                
                # 添加边界框坐标
                bbox_text = f"Box: ({box[0]}, {box[1]}, {box[2]}, {box[3]})"
                cv2.putText(annotated_frame,
                          bbox_text,
                          (box[0], box[3] + 20),  # 文本位置在边界框下方
                          cv2.FONT_HERSHEY_SIMPLEX,
                          0.5,
                          (0, 255, 0),
                          2)
            
            # 将处理后的图像转换回 ROS 消息并发布
            detection_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.detection_pub.publish(detection_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 