import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import QoSProfile
import math
import time

class LegWrenchPublisher(Node):
    def __init__(self, publish_interval=1.0):
        super().__init__('apply_wrench')

        # Time interval (seconds) between each publish
        self.publish_interval = publish_interval

        # Quality of Service profile for reliability and durability
        qos_profile = QoSProfile(depth=10)

        # Create publishers for each leg
        self.front_left_publisher = self.create_publisher(WrenchStamped, '/front_left_foot/wrench', qos_profile)
        self.front_right_publisher = self.create_publisher(WrenchStamped, '/front_right_foot/wrench', qos_profile)
        self.rear_left_publisher = self.create_publisher(WrenchStamped, '/rear_left_foot/wrench', qos_profile)
        self.rear_right_publisher = self.create_publisher(WrenchStamped, '/rear_right_foot/wrench', qos_profile)

        # Create a timer to publish wrenches at the specified interval
        self.timer = self.create_timer(self.publish_interval, self.publish_wrenches)

    def publish_wrenches(self):
        # Define a sample wrench force and torque for each foot
        # You could also vary these values over time
        front_left_wrench = self.create_wrench(force_x=0.0, force_y=0.0, force_z=0.1, torque_x=0.0, torque_y=0.0, torque_z=0.0)
        front_right_wrench = self.create_wrench(force_x=0.0, force_y=0.0, force_z=0.1, torque_x=0.0, torque_y=0.0, torque_z=0.0)
        rear_left_wrench = self.create_wrench(force_x=0.0, force_y=0.0, force_z=0.1, torque_x=0.0, torque_y=0.0, torque_z=0.0)
        rear_right_wrench = self.create_wrench(force_x=0.0, force_y=0.0, force_z=0.1, torque_x=0.0, torque_y=0.0, torque_z=0.0)

        # Publish each wrench message to the respective leg topic
        self.front_left_publisher.publish(front_left_wrench)
        self.front_right_publisher.publish(front_right_wrench)
        self.rear_left_publisher.publish(rear_left_wrench)
        self.rear_right_publisher.publish(rear_right_wrench)

        self.get_logger().info("Published wrench")

    def create_wrench(self, force_x=0.0, force_y=0.0, force_z=0.0, torque_x=0.0, torque_y=0.0, torque_z=0.0):
        """Helper function to create a WrenchStamped message with specified force and torque values."""
        wrench = WrenchStamped()
        wrench.header.stamp = self.get_clock().now().to_msg()
        wrench.header.frame_id = 'base_link'  # Replace with the appropriate frame if needed

        wrench.wrench.force.x = force_x
        wrench.wrench.force.y = force_y
        wrench.wrench.force.z = force_z
        wrench.wrench.torque.x = torque_x
        wrench.wrench.torque.y = torque_y
        wrench.wrench.torque.z = torque_z

        return wrench


def main(args=None):
    rclpy.init(args=args)
    leg_wrench_publisher = LegWrenchPublisher(publish_interval=5.0)  # Adjust interval as needed

    try:
        rclpy.spin(leg_wrench_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        leg_wrench_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
