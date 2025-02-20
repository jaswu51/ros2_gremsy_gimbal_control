import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Image Viewer', frame)
            cv2.waitKey(1)  # Wait for a key press for 1 ms
        except Exception as e:
            self.get_logger().error('Error converting image: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()