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
        super().__init__('hw5_pub')

        self.ranges = []
        self.state = State.STRAIGHT
        self.last_state = State.STRAIGHT
        self.start_count = 0
        self._counter = 0


        self.gazebo = True
        
        # !----- MODULAR VALUES -----! #
        self.THETA = 5
        self.BUFFER = 0.05
        self.STRAIGHT_BUFFER = 0.1
        self.FRONT_WALL_DIST = .5
        self.RIGHT_WALL_DIST = .55
        self.MAX_DIST = 0.8
        self.COUNTER_DIFF = 10

        self.FRONT = 360
        self.LEFT = 540
        self.RIGHT = 180
# !--------------------------! #
        if self.gazebo:
            self.THETA = 3
            self.FRONT = 0
            self.LEFT = 90
            self.RIGHT = 270
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 100)
        self.pose_sub = self.create_subscription(LaserScan, "/scan", self.pranav_update_ranges, 100)
        self.timer = self.create_timer(0.01, self.pranav_timer_callback)
        self.scan_msg = None
        self.current_angle = 0
        self.last_time = 0
        self.elapsed_time = 0
        self.turn_switch = False
        self.desired_turn_time = 11.2
        self.has_started = False
        
    def has_right(self, right_dist, top_right_dist):
        return right_dist < self.RIGHT_WALL_DIST or top_right_dist < self.RIGHT_WALL_DIST
    
    def has_front(self, front_dist):
        return front_dist < self.FRONT_WALL_DIST
    
    def new_update_ranges(self, scan_msg):
        front = scan_msg.ranges[self.FRONT]
        right = scan_msg.ranges[self.RIGHT]  
        top_right = scan_msg.ranges[self.RIGHT+30]
        
        if self.has_started:
            ### TRUTH TABLE ###
            # If there is nothing to the right and you are already inside of the maze, if there is nothing on the right, you turn CW
            if not self.turn_switch and not self.has_right(right, top_right):
                if self.last_state != self.state:
                    self.get_logger().info("Turning RIGHT")
                self.state = State.TURNING_RIGHT
            
            # If there is something to your right, but nothing to your front, drive straight.
            elif not self.turn_switch and self.has_right(right, top_right) and not self.has_front(front):
                if self.last_state != self.state:
                    self.get_logger().info("Going Straight")
                self.state = State.STRAIGHT
            
            #if there is something to your right and something to your front, you need to drive counter clockwise.
            elif not self.turn_switch and self.has_right(right, top_right) and self.has_front(front):
                if self.last_state != self.state:
                    self.get_logger().info("Turning LEFT")
                self.state = State.TURNING_LEFT
            else:
                self.get_logger().info("Something went wrong!")
                self.get_logger().info("Right: "+str(self.has_right(right, top_right)))
                self.get_logger().info("Front: "+str(self.has_front(front)))
        else:
            self.get_logger().info("We have not started!")
            if self.has_front(front):
                self.get_logger().info("We have started!")
                self.has_started = True
            self.state = State.STRAIGHT
            
    def pranav_update_ranges(self, scan_val):
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
            
    def pranav_timer_callback(self):
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
                 
    def timer_callback(self):
        LINEAR = .3
        ANGULAR = .5
        angular_speed = ANGULAR
        self.get_logger().info("elapsed time"+str(self.elapsed_time))
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        if self.last_state == State.STRAIGHT and (self.state == State.TURNING_LEFT or self.state == State.TURNING_RIGHT):
            self.get_logger().info("Turning switch to true")
            self.turn_switch = True
            self.elapsed_time = 0
        
        if self.turn_switch and self.elapsed_time >= self.desired_turn_time:
            self.get_logger().info("Going straight")
            self.state = State.STRAIGHT
            self.turn_switch = False
        
        if self.state == State.STRAIGHT:
            if self.last_state != self.state:
                self.get_logger().info("Going straight")
            msg.linear.x = LINEAR
        elif self.state == State.TURNING_LEFT:
            if self.last_state != self.state:
                self.get_logger().info("Going left")
            msg.angular.z = abs(angular_speed)
        elif self.state == State.TURNING_RIGHT:
            if self.last_state != self.state:
                self.get_logger().info("Going right")
            msg.angular.z = -abs(angular_speed)
        else:
            self.get_logger().info("state error")
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher.publish(msg)
        
        curr_time = self.get_clock().now().seconds_nanoseconds()
        curr_time = curr_time[0] + curr_time[1] / 1000000000.0
        self.elapsed_time += curr_time - self.last_time
        self.last_time = curr_time
        self.last_state = self.state
            

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
    node.destroy_node()

if __name__ == '__main__':
    main()