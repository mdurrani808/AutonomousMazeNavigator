import rclpy
from rclpy.node import Node
from enum import Enum
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class State(Enum):
    STOPPED: int = 0,
    STRAIGHT: int = 1,
    TURNING_LEFT: int = 2,
    TURNING_RIGHT: int = 3,
    STRAIGHT_LEFT: int = 4,
    STRAIGHT_RIGHT: int = 5,

def mean(arr):
    arr = [i for i in arr if i != math.inf]
    if len(arr) > 0:
        return sum(arr) / len(arr)
    else:
        return math.inf

class Homework5(Node):
    def __init__(self):
        super().__init__('hw35_pub')

        self.ranges = []
        self.state = State.STRAIGHT
        self.start_count = 0
        self._counter = 0

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 100)
        self.pose_sub = self.create_subscription(LaserScan, "/scan", self.update_ranges, 100)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.gazebo = False
    def update_ranges(self, scan_val):
        self.ranges = scan_val.ranges
        self._counter += 1
        # !----- MODULAR VALUES -----! #
        THETA = 5
        BUFFER = 0.1
        STRAIGHT_BUFFER = 0.1
        WALL_DIST = 0.4
        MAX_DIST = 0.9
        COUNTER_DIFF = 10

        FRONT = 360
        LEFT = 540
        RIGHT = 180
        # !--------------------------! #
        if self.gazebo:
            THETA = 3
            FRONT = 180
            LEFT = 270
            RIGHT = 90


        front_dist = mean(self.ranges[FRONT - THETA : FRONT] + self.ranges[FRONT : FRONT + THETA])
        left_dist = mean(self.ranges[LEFT - THETA : LEFT + THETA])
        front_left_dist = mean(self.ranges[LEFT - THETA * 3 : LEFT - THETA])
        back_left_dist = mean(self.ranges[LEFT + THETA : LEFT + THETA * 3])
        diagonal_left_dist = mean(self.ranges[(FRONT + LEFT) // 2 - THETA : (FRONT + LEFT) // 2 + THETA])

        if front_left_dist < back_left_dist:
            self.state = State.TURNING_RIGHT
        # If parallel and far away from front wall keep going straight
        if abs(front_left_dist - back_left_dist) < BUFFER and front_dist > WALL_DIST:
            self.state = State.STRAIGHT
        # If too close to front wall and left wall, turn right or if back left is greater than front left we need to turn right to adjust
        elif (front_dist < WALL_DIST and diagonal_left_dist < MAX_DIST) or (back_left_dist - front_left_dist) < BUFFER:

            self.state = State.TURNING_RIGHT
        else:
            self.state = State.TURNING_LEFT

    def timer_callback(self):
        LINEAR = 0.5
        ANGULAR = 0.25
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        if self.state == State.STRAIGHT:
            msg.linear.x = LINEAR
        elif self.state == State.TURNING_LEFT:
            msg.angular.z = ANGULAR
        elif self.state == State.TURNING_RIGHT:
            msg.angular.z = -ANGULAR
        else:
            msg.linear.x = 0.0
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Homework5()
    rclpy.spin(node)
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.y = 0.0
    node.publisher.publish(msg)

rclpy.shutdown()

if __name__ == '__main__':
    main()