import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Imu
import math
import time
from tf_transformations import euler_from_quaternion

class SpinTimeCalculator(Node):
    def __init__(self):
        super().__init__('spin_time_calculator')
        self.pose_sub = self.create_subscription(Imu, '/imu', self.pose_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_yaw = None
        self.start_time = None
        self.end_time = None
        self.target_angle = math.pi / 2  # 90 degrees in radians

    def pose_callback(self, msg):
        if self.initial_yaw is None:
            orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            self.initial_yaw = yaw
            self.start_time = time.time()
            self.spin_turtlebot()
        else:
            orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            current_angle = yaw -self.initial_yaw
            self.get_logger().info("Angle is "+str(current_angle))
            self.get_logger().info("Targer Angle:"+str(self.target_angle))
            if current_angle >= self.target_angle:
                self.end_time = time.time()
                self.stop_turtlebot()
                spin_duration = self.end_time - self.start_time
                self.get_logger().info(f"Turtlebot completed 90 degrees spin in {spin_duration:.2f} seconds.")
                self.destroy_node()

    def spin_turtlebot(self):
        twist_msg = Twist()
        twist_msg.angular.z = 0.5  # Adjust the angular velocity as needed
        self.cmd_vel_pub.publish(twist_msg)

    def stop_turtlebot(self):
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)

    def calculate_angle(self, pose1, pose2):
        return abs(pose1.orientation.z - pose2.orientation.z)

def main(args=None):
    rclpy.init(args=args)
    node = SpinTimeCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()