import rclpy
from rclpy.node import Node
from enum import Enum
import rclpy.time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class State(Enum):
    STOPPED: int = 0,
    STRAIGHT: int = 1,
    TURNING_LEFT: int = 2,
    TURNING_RIGHT: int = 3,
    PARALLELIZING: int = 4,
    STRAIGHT_AFTER_RIGHT: int = 5,
    SPINNING: int = 6

class Homework5(Node):
    def __init__(self):
        super().__init__('hw5_pub')

        self.state = State.STRAIGHT
        self.last_state = State.STRAIGHT
        self.last_turn_state = None
        self.elapsed_time = 0
        self.last_time = 0
        self.turn_switch = False
        self.desired_turn_time = 30.0
        self.has_started = False
        self.parallelizing_time = 2.0
        self.tolerance = 0.005
        self.parallelizing_error = 0
        self.max_parallelizing_effort = 1.0
        self.straight_after_right_time = 5.0
        
        # !----- MODULAR VALUES -----! #
        self.FRONT_WALL_DIST = .5
        self.RIGHT_WALL_DIST = .55
        self.LONG_RIGHT_WALL_DIST = .75
        self.FRONT = 0
        self.LEFT = 90
        self.RIGHT = 270
        self.THETA = 10
        self.parallelBuffer = 0.015
        # !--------------------------! #
        
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 100)
        self.pose_sub = self.create_subscription(LaserScan, "/scan", self.update_ranges, 100)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
    def has_right(self, right_dist, top_right_dist, bottom_right_dist):
        return right_dist < self.RIGHT_WALL_DIST or bottom_right_dist < self.LONG_RIGHT_WALL_DIST
    
    def has_front(self, front_dist):
        return front_dist < self.FRONT_WALL_DIST
    
    def is_parallel(self, topRightRange, bottomRightRange, middleRightRange):
        return abs(topRightRange - bottomRightRange) < self.parallelBuffer and middleRightRange <= min(topRightRange, bottomRightRange)
    
    def mean(self, arr):
        return sum(arr) / len(arr)
    
    def update_ranges(self, scan_msg):
        rangesArr = scan_msg.ranges
        front = rangesArr[self.FRONT]
        right = rangesArr[self.RIGHT]  
        top_right = rangesArr[self.RIGHT+30]
        left = rangesArr[self.LEFT]
        middleRanges = rangesArr[-5:]
        topRightRanges = rangesArr[250:260]
        middleRightRanges = rangesArr[265:275]
        bottomRightRanges = rangesArr[280:290]
        megaTopRightRanges = rangesArr[240:250]

        middleRange = self.mean(middleRanges)
        topRightRange = self.mean(topRightRanges)
        middleRightRange = self.mean(middleRightRanges)
        bottomRightRange = self.mean(bottomRightRanges)
        megaTopRightRange = self.mean(megaTopRightRanges)

        if self.has_started:
            if self.state == State.SPINNING:
                return
            
            if self.state == State.PARALLELIZING:
                print("topRight: " + str(topRightRange))
                print("bottomRight: " + str(bottomRightRange))
                print("top-bottom right: " + str(abs(topRightRange - bottomRightRange)))

                if self.is_parallel(topRightRange, bottomRightRange, middleRightRange):
                    self.state = State.STRAIGHT
                    self.get_logger().info(f"Transitioning from {self.last_state} to {self.state}")
            
            elif self.state == State.STRAIGHT_AFTER_RIGHT:
                if self.elapsed_time >= self.straight_after_right_time:
                    self.state = State.STRAIGHT
                    self.get_logger().info(f"Transitioning from {self.last_state} to {self.state}")
                        
            elif not self.turn_switch and not self.has_right(right, top_right, megaTopRightRange):
                self.state = State.TURNING_RIGHT
                self.last_turn_state = State.TURNING_RIGHT
                self.get_logger().info(f"Transitioning from {self.last_state} to {self.state}")
            
            elif not self.turn_switch and (self.has_right(right, top_right, bottomRightRange) and not self.has_front(front)):
                self.state = State.STRAIGHT
                self.get_logger().info(f"Transitioning from {self.last_state} to {self.state}")
            
            elif not self.turn_switch and self.has_right(right, top_right, bottomRightRange) and self.has_front(front):
                self.state = State.TURNING_LEFT
                self.last_turn_state = State.TURNING_LEFT
                self.get_logger().info(f"Transitioning from {self.last_state} to {self.state}")
            
            if math.isinf(left) and math.isinf(right):
                self.state = State.SPINNING
                self.get_logger().info(f"Transitioning from {self.last_state} to {self.state}")
        else:
            if self.has_front(front):
                self.has_started = True
                self.get_logger().info("Rover has started")
            self.state = State.STRAIGHT
            self.get_logger().info(f"Transitioning from {self.last_state} to {self.state}")
            
   
    def timer_callback(self):
        LINEAR = .25
        ANGULAR = .1
        msg = Twist()
        
        if self.last_state == State.STRAIGHT and (self.state == State.TURNING_LEFT or self.state == State.TURNING_RIGHT):
            self.turn_switch = True
            self.elapsed_time = 0
            self.get_logger().info("Turn switch set to True")
        
        if self.turn_switch and self.elapsed_time >= self.desired_turn_time:
            if self.last_turn_state == State.TURNING_LEFT:
                self.state = State.PARALLELIZING
            else:
                self.state = State.STRAIGHT_AFTER_RIGHT
                self.elapsed_time = 0
            self.turn_switch = False
            self.get_logger().info(f"Transitioning from {self.last_state} to {self.state}")
        
        if self.state == State.STRAIGHT:
            msg.linear.x = LINEAR
        elif self.state == State.TURNING_LEFT:
            msg.angular.z = ANGULAR
        elif self.state == State.TURNING_RIGHT:
            msg.angular.z = -ANGULAR
        elif self.state == State.STOPPED:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        elif self.state == State.PARALLELIZING:
            parallelizing_effort = self.parallelizing_error / self.tolerance
            msg.angular.z = (ANGULAR) * self.sign(parallelizing_effort) * -1
        elif self.state == State.STRAIGHT_AFTER_RIGHT:
            msg.linear.x = LINEAR
        elif self.state == State.SPINNING:
            msg.angular.z = ANGULAR
        
        self.publisher.publish(msg)
        
        curr_time = self.get_clock().now().seconds_nanoseconds()
        curr_time = curr_time[0] + curr_time[1] / 1000000000.0
        self.elapsed_time += curr_time - self.last_time
        self.last_time = curr_time
        self.last_state = self.state
            
    def sign(self,num):
        return -1 if num < 0 else 1

def main(args=None):
    rclpy.init(args=args)
    
    node = Homework5()
    rclpy.spin(node)
    
    msg = Twist()
    node.publisher.publish(msg)

    rclpy.shutdown()
    node.destroy_node()

if __name__ == '__main__':
    main()