import rclpy
from rclpy.node import Node
from enum import Enum
import rclpy.time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class Motion_State(Enum):
    STOPPED: int = 0,
    STRAIGHT: int = 1,
    TURNING_LEFT: int = 2,
    TURNING_RIGHT: int = 3,
    PARALLELIZING: int = 4,
    STRAIGHT_NO_PARALLELIZING: int = 5,
    SPINNING: int = 6

class Actions(Enum):
    STRAIGHT: int = 0,
    RIGHT_TURN: int = 1,
    LEFT_TURN: int = 2,
    SPIN: int = 3,
    PARALLELIZE: int = 4

class Homework5(Node):
    def __init__(self):
        super().__init__('hw5_pub')

        self.state = Motion_State.STRAIGHT
        self.last_state = Motion_State.STRAIGHT
        self.current_action = Actions.STRAIGHT
        self.last_turn_state = None
        self.elapsed_time = 0
        self.last_time = 0
        self.turn_switch = False
        self.desired_turn_time = 30.0
        self.has_started = False
        self.parallelizing_time = 2.0
        self.tolerance = 0.0075
        self.max_parallelizing_effort = 1.0
        self.straight_after_right_time = 3.5
        self.left_counter = 0
        self.right_counter = 0
        self.counter = 0
        self.wall_parallelize = ""
        self.hugging_right_wall=False
        self.cleared_front_wall=True
        self.gazebo = False


        # !----- MODULAR VALUES -----! #
        
        if(self.gazebo):
            self.angle_range = 3
            self.FRONT_WALL_DIST = .45

            self.MIN_RIGHT_WALL_DIST = 0.3
            self.MAX_RIGHT_WALL_DIST = 0.4

            self.FRONT = 0
            self.LEFT = 90
            self.RIGHT = 270
            self.THETA = 10
            self.parallelBuffer = 0.01
            self.RANGE_MIN_IND = 0
            self.RANGE_MAX_IND = 360

            self.RIGHT_FRONT = 285
            self.RIGHT_BACK = 255

            self.parallelizing_error = 0.005
            self.RIGHT_SMALLER_PAR_RANGE = 10
            self.CURR_PAR_RANGE = 10
            self.RIGHT_TURN_SIGNAL_ANGLE = 330
            self.RIGHT_TURN_BUFFER = 0.3
            self.parallelizing_direction = 0
        else:
            self.angle_range = 6
            self.FRONT_WALL_DIST = 0.35

            self.MIN_RIGHT_WALL_DIST = 0.2
            self.MAX_RIGHT_WALL_DIST = 0.55

            self.FRONT = 360
            self.LEFT = 540
            self.RIGHT = 180
            self.THETA = 10
            self.parallelBuffer = 0.01
            self.RANGE_MIN_IND = 0
            self.RANGE_MAX_IND = 720

            self.RIGHT_FRONT = 200 # was 285
            self.RIGHT_BACK = 160

            self.parallelizing_error = 0.008
            self.RIGHT_SMALLER_PAR_RANGE = 10
            self.CURR_PAR_RANGE = 10
            self.RIGHT_TURN_SIGNAL_ANGLE = 300 # was 330
            self.RIGHT_TURN_BUFFER = 0.3
            self.parallelizing_direction = 0

            self.RIGHT_TURN_STRAIGHT_COUNT = 13
        # !--------------------------! #

        
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 100)
        self.pose_sub = self.create_subscription(LaserScan, "/scan", self.update_ranges, 100)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
    def has_right(self, right_dist, top_right_dist, bottom_right_dist):
        return right_dist < self.RIGHT_WALL_DIST or bottom_right_dist < self.LONG_RIGHT_WALL_DIST
    def has_left(self, left_dist, top_left_dist, bottom_left_dist):
        return left_dist < self.RIGHT_WALL_DIST or bottom_left_dist < self.LONG_RIGHT_WALL_DIST
    

    def has_front(self, front_dist):
        return front_dist < self.FRONT_WALL_DIST
    
    def is_parallel(self, topRightRange, bottomRightRange, middleRightRange, topLeftRange, bottomLeftRange, middleLeftRange):
        if(abs(topRightRange - bottomRightRange) < abs(topLeftRange - bottomLeftRange) and self.wall_parallelize == ""):
            self.wall_parallelize = "Right"
        elif(abs(topRightRange - bottomRightRange) > abs(topLeftRange - bottomLeftRange) and self.wall_parallelize == ""):
            self.wall_parallelize = "Left"
        if(abs(topRightRange - bottomRightRange) < self.parallelBuffer and middleRightRange <= min(topRightRange, bottomRightRange)): 
            return True
        elif(abs(topLeftRange - bottomLeftRange) < self.parallelBuffer and middleLeftRange <= min(topLeftRange, bottomLeftRange)):
            return True
        return False
    
    def mean(self, arr):
        arr = [i for i in arr if i != math.inf]
        if len(arr) > 0:
            return sum(arr) / len(arr)
        else:
            return math.inf
    
    def update_ranges(self, scan_msg):
        self.counter += 1
        rangesArr = scan_msg.ranges
        self.ranges = rangesArr

        front = self.get_dist_at_angle(self.FRONT)
        left = self.get_dist_at_angle(self.LEFT)
        right = self.get_dist_at_angle(self.RIGHT)
        right_front_dist = self.get_dist_at_angle(self.RIGHT_FRONT, range_size=2)
        right_back_dist = self.get_dist_at_angle(self.RIGHT_BACK, range_size=2)
        parallel_error = right_front_dist - right_back_dist
        right_signal_dist = self.get_dist_at_angle(self.RIGHT_TURN_SIGNAL_ANGLE)
        
        print(self.get_angle_range(self.RIGHT_FRONT))
        print(right_front_dist)
            
   
    def get_angle_range(self, angle, range_size=None):
        range_arr = []
        if range_size == None:
            range_size = self.angle_range

        i = angle - range_size
        while i < angle + range_size:
            index = i
            if index < self.RANGE_MIN_IND:
                index += self.RANGE_MAX_IND
            elif index > self.RANGE_MAX_IND:
                index -= self.RANGE_MAX_IND
            range_arr.append(self.ranges[index])
            i += 1

        return range_arr
    

    def get_dist_at_angle(self, angle, range_size=None):
        range_dist = self.get_angle_range(angle, range_size)
        return self.mean(range_dist)


    def timer_callback(self):
        LINEAR = .15
        ANGULAR = .3
        msg = Twist()
                
        0.0
        
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