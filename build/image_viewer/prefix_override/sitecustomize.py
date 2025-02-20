import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dtc-mrsd/Downloads/ros2_gimbal_ws/install/image_viewer'
