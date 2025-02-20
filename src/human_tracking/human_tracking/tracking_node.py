#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
import math

class HumanTrackingNode(Node):
    def __init__(self):
        super().__init__('human_tracking_node')
        
        # 订阅检测中心点
        self.center_sub = self.create_subscription(
            Point,
            'detection_center',
            self.center_callback,
            10
        )
        
        # 发布云台控制命令
        self.gimbal_pub = self.create_publisher(
            Vector3,
            'gimbal_angles',
            10
        )
        
        # 图像中心点（1280x720 分辨率）
        self.image_center_x = 1280 / 2  # 640
        self.image_center_y = 720 / 2   # 360
        
        # PID 控制参数
        self.kp_x = 0.05  # 上下运动的比例系数
        self.kp_z = 0.05  # 左右运动的比例系数
        
        # 死区阈值（像素）
        self.deadzone = 30
        
        # 当前云台角度
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        self.get_logger().info('Human tracking node initialized')

    def center_callback(self, msg):
        try:
            # 计算图像中心点和检测中心点的偏差
            error_x = msg.y - self.image_center_y  # 注意：y对应pitch（上下）
            error_z = msg.x - self.image_center_x  # 注意：x对应yaw（左右）
            
            # 如果偏差在死区内，不进行调整
            if abs(error_x) < self.deadzone:
                error_x = 0
            if abs(error_z) < self.deadzone:
                error_z = 0
            
            # 计算云台角度调整量
            delta_pitch = -self.kp_x * error_x  # 上下运动（负号是因为坐标系方向）
            delta_yaw = -self.kp_z * error_z    # 左右运动
            
            # 更新云台角度
            self.current_pitch = self.current_pitch + delta_pitch
            self.current_yaw = self.current_yaw + delta_yaw
            
            # 限制云台角度范围
            self.current_pitch = max(min(self.current_pitch, 45), -45)  # 限制俯仰角范围
            self.current_yaw = max(min(self.current_yaw, 180), -180)   # 限制偏航角范围
            
            # 创建并发布云台控制命令
            gimbal_cmd = Vector3()
            gimbal_cmd.x = 0.0                    # roll
            gimbal_cmd.y = self.current_pitch     # pitch
            gimbal_cmd.z = self.current_yaw       # yaw
            
            self.gimbal_pub.publish(gimbal_cmd)
            
            # 记录日志
            if abs(error_x) > self.deadzone or abs(error_z) > self.deadzone:
                self.get_logger().info(
                    f'Tracking: error_x={error_x:.1f}, error_z={error_z:.1f}, '
                    f'pitch={self.current_pitch:.1f}, yaw={self.current_yaw:.1f}'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error in tracking: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = HumanTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 