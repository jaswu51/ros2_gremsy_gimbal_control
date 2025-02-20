# ROS2 Gimbal Control Workspace

## Build and Run the Workspace
```bash
cd ros2_gremsy_gimbal_control
colcon build --cmake-args -DGHADRON=1
source install/setup.bash
```

## Function moduels

### Gimbal bringup
launch all the packages in the gimbal system
```bash
ros2 launch gimbal_bringup gimbal_system.launch.py
```
The rest of the nodes are automatically launched by gimbal_system.launch.py

### Gimbal Angle Control
```bash
ros2 run gimbal_angle_control gimbal_angle_control_node
```
Send control commands (roll=0, pitch=0, yaw=90)
```bash
ros2 topic pub /gimbal_angles geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 90.0}"
```

### Get gimbal status
```bash
ros2 run gimbal_status gimbal_status_node
```

View topic information
```bash
ros2 topic info /gimbal_angles
ros2 topic info /gimbal_zoom
```

### Zoom control for ir camera
```bash
ros2 run ir_zoom ir_zoom_node
```
set zoom in/out/stop and zoom range for ir camera
```bash
ros2 topic pub /ir_zoom/command std_msgs/msg/String "{data: 'zoom_in'}"
ros2 topic pub /ir_zoom/command std_msgs/msg/String "{data: 'stop'}"
ros2 topic pub /ir_zoom/command std_msgs/msg/String "{data: 'zoom_out'}"
ros2 topic pub /ir_zoom/range std_msgs/msg/Float32 "{data: 50.0}" # 50%. between 0.0 and 100.0
```

### Zoom control for eo camera
```bash
ros2 run eo_zoom eo_zoom_node
```
set zoom in/out/stop and zoom range for eo camera
```bash
ros2 topic pub /eo_zoom/command std_msgs/msg/String "{data: 'zoom_in'}"
ros2 topic pub /eo_zoom/command std_msgs/msg/String "{data: 'stop'}"
ros2 topic pub /eo_zoom/command std_msgs/msg/String "{data: 'zoom_out'}"
ros2 topic pub /eo_zoom/range std_msgs/msg/Float32 "{data: 50.0}" # 50%. between 0.0 and 100.0
```

### YOLO detection
```bash
ros2 run yolo_detection yolo_detector_node
```
check the published topics
```bash
ros2 topic info /detection_box
ros2 topic info /detection_box_center
```
### Human tracking
```bash
ros2 run human_tracking tracking_node
```
check the published topics
```bash
ros2 topic info /gimbal_angles
```
## Other functions
### Check the streaming video
image_publisher is a node that publishes the streaming video to the topic /image_raw, image_viewer is a node that subscribes to the topic /image_raw and displays the video. We don't use them for now because the streaming video contains ir and eo overlapped and not being able to display separately.
```bash
ros2 run image_publisher image_publisher_node rtsp://10.3.1.124:8554/ghadron
ffmpeg -y -i rtsp://10.3.1.124:8554/ghadron 1 do.jpg
ros2 topic echo /image_raw
```

### hard-coded path prefix
needed to be changed to if code is migrated to other machine
```bash
/home/dtc-mrsd/Downloads/ros2_gimbal_ws/
```

