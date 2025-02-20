from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=['Starting Gimbal System...']),
        
        # 云台控制节点
        Node(
            package='gimbal_angle_control',
            executable='gimbal_angle_control_node',
            name='gimbal_angle_control_node',
            output='screen'
        ),
        
        # 云台状态节点
        Node(
            package='gimbal_status',
            executable='gimbal_status_node',
            name='gimbal_status_node',
            output='screen'
        ),
        
        # EO变焦节点
        Node(
            package='eo_zoom',
            executable='eo_zoom_node',
            name='eo_zoom_node',
            output='screen'
        ),
        
        # IR变焦节点
        Node(
            package='ir_zoom',
            executable='ir_zoom_node',
            name='ir_zoom_node',
            output='screen'
        ),
        
        # YOLO检测节点
        Node(
            package='yolo_detection',
            executable='yolo_detector',
            name='yolo_detector_node',
            output='screen'
        ),
        
        # 人物追踪节点
        Node(
            package='human_tracking',
            executable='tracking_node',
            name='tracking_node',
            output='screen'
        ),
        
        LogInfo(msg=['All nodes have been started.'])
    ]) 