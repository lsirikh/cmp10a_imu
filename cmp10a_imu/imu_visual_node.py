import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker

class IMUVisualNode(Node):
    def __init__(self):
        super().__init__('imu_visual_node')

        # Subscribe to the IMU data topic
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher for visualization markers
        self.publisher = self.create_publisher(Marker, '/visualization_marker', 10)

        # Initialize the marker
        self.marker = Marker()
        self.marker.header.frame_id = "base_link"  # Ensure this matches your frame
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.scale.x = 1.0  # Adjust the size as needed
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.2
        self.marker.color.a = 1.0  # Don't forget to set the alpha!
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

    def imu_callback(self, msg):
        # Extract orientation from IMU message
        orientation = msg.orientation
        self.marker.header.stamp = self.get_clock().now().to_msg()

        # Update marker orientation
        self.marker.pose.orientation = orientation

        # Set the position if needed
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0

        # Publish the marker
        self.publisher.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    node = IMUVisualNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
