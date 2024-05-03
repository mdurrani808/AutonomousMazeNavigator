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
        self.FRONT_WALL_DIST = 1.25
        self.RIGHT_WALL_DIST = .55
        self.MAX_DIST = 0.9
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
        self.pose_sub = self.create_subscription(LaserScan, "/scan", self.new_update_ranges, 100)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.scan_msg = None
        self.current_angle = 0
        self.last_time = 0
        self.elapsed_time = 0
        self.turn_switch = False
        self.desired_turn_time = 21.0
        self.has_started = False
    def update_ranges(self, scan_val):
        self.ranges = scan_val.ranges
        self._counter += 1


        front_dist = mean(self.ranges[self.FRONT - self.THETA : self.FRONT] + self.ranges[self.FRONT : self.FRONT + self.THETA])
        
        left_dist = mean(self.ranges[self.LEFT - self.THETA : self.LEFT + self.THETA])
        front_left_dist = mean(self.ranges[self.LEFT - self.THETA * 3 : self.LEFT - self.THETA])
        back_left_dist = mean(self.ranges[self.LEFT + self.THETA : self.LEFT + self.THETA * 3])
        diagonal_left_dist = mean(self.ranges[(self.FRONT + self.LEFT) // 2 - self.THETA : (self.FRONT + self.LEFT) // 2 + self.THETA])

        if self._counter - self.start_count >= self.COUNTER_DIFF:
            if diagonal_left_dist >= self.MAX_DIST:
                self.state = State.TURNING_LEFT
                self.start_count = self._counter
                self.get_logger().info("turning left " +  str(self._counter))
            elif front_dist < self.WALL_DIST:
                self.state = State.TURNING_RIGHT
                self.start_count = self._counter
                self.get_logger().info("turning right " + str(self._counter))
            # straight with wall
            elif abs(front_left_dist - back_left_dist) < self.BUFFER and left_dist <= min(front_left_dist, back_left_dist) < self.WALL_DIST:
                self.state = State.STRAIGHT
                #self.get_logger().info(f"going straight ({str(self._counter)}) {abs(front_left_dist - back_left_dist)} {left_dist} <= {min(front_left_dist, back_left_dist)}")

        if self.state == State.STRAIGHT:
            if front_left_dist - back_left_dist > self.STRAIGHT_BUFFER:
                self.state = State.STRAIGHT_LEFT
                self.get_logger().info("straight left")
            elif back_left_dist - front_left_dist > self.STRAIGHT_BUFFER:
                self.state = State.STRAIGHT_RIGHT
                self.get_logger().info("straight right")

    def has_right(self, right_dist, top_right_dist):
        return right_dist < self.RIGHT_WALL_DIST or top_right_dist < self.RIGHT_WALL_DIST
    
    def has_left(self, left_dist):
        return left_dist < self.RIGHT_WALL_DIST
    def has_front(self, front_dist):
        return front_dist < self.FRONT_WALL_DIST
    
    def is_parallel(self, front_left_dist, back_left_dist, left_dist):
        return abs(front_left_dist - back_left_dist) < self.BUFFER and left_dist <= min(front_left_dist, back_left_dist) < self.FRONT_WALL_DIST
    
    def new_update_ranges(self, scan_msg):
        front = scan_msg.ranges[self.FRONT]
        left = mean(scan_msg.ranges[self.LEFT - self.THETA : self.LEFT + self.THETA])
        front_left_dist = mean(scan_msg.ranges[self.LEFT - self.THETA * 3 : self.LEFT - self.THETA])
        back_left_dist = mean(scan_msg.ranges[self.LEFT + self.THETA : self.LEFT + self.THETA * 3])
        top_left = scan_msg.ranges[45]  
        right = scan_msg.ranges[self.RIGHT]  
        self.get_logger().info(str(right))
        self.get_logger().info(str(self.elapsed_time))
        
        top_right = scan_msg.ranges[self.RIGHT+30]
        self.get_logger().info(str(self.has_right(right, top_right)))
        
        if(self.has_started):
            ### TRUTH TABLE ###
            # If there is nothing to the right and you are already inside of the maze, if there is nothing on the right, you turn CW
            if(not self.turn_switch and not self.has_right(right, top_right)):
                if(self.last_state != self.state):
                    self.get_logger().info("Turning LEFT")
                self.state = State.TURNING_RIGHT
            
            # If there is something to your right, but nothing to your front, drive straight.
            elif(self.has_right(right, top_right) and not self.has_front(front)):
                if(self.last_state != self.state):
                    self.get_logger().info("Going Straight")
                self.state = State.STRAIGHT
            
            #if there is something to your right and something to your front, you need to drive counter clockwise.
            elif(not self.turn_switch and self.has_right(right, top_right) and self.has_front(front)):
                if(self.last_state != self.state):
                    self.get_logger().info("Turning Right")
                self.state = State.TURNING_LEFT
            else:
                self.get_logger().info("Something went wrong!")
                self.get_logger().info("Right: "+str(self.has_right(right, top_right)))
                self.get_logger().info("Front: "+str(self.has_front(front)))
        else:
            self.get_logger().info("We have not started!")
            if(self.has_front(front)):
                self.get_logger().info("We have started!")
                self.has_started = True
            self.state = State.STRAIGHT
            
        
    def timer_callback(self):
        LINEAR = 1.0
        ANGULAR = 12.0
        desired_angle = 90
        curr_time = self.get_clock().now().seconds_nanoseconds()
        curr_time = curr_time[0] + curr_time[1] / 1000000000.0

        angular_speed = round(ANGULAR*2*math.pi/360, 1)
        relative_angle = round(desired_angle*2*math.pi/360, 1)
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        
        if(self.last_state == State.STRAIGHT and (self.state == State.TURNING_LEFT or self.state == State.TURNING_RIGHT)):
            self.get_logger().info("Turning switch to true")
            self.turn_switch = True
        if(self.turn_switch and ((self.elapsed_time >= self.desired_turn_time))):
            self.get_logger().info("Going straight")
            self.state = State.STRAIGHT
            self.turn_switch = False
        if self.state == State.STRAIGHT:
            if(self.last_state != self.state):
                self.get_logger().info("Going straight")
            msg.linear.x = LINEAR
        elif self.state == State.TURNING_LEFT:
            if(self.last_state != self.state):
                self.get_logger().info("Going left")
            msg.angular.z = abs(angular_speed)
        elif self.state == State.TURNING_RIGHT:
            if(self.last_state != self.state):
                self.get_logger().info("Going right")
            msg.angular.z = -abs(angular_speed)
        else:
            self.get_logger().info("state error")
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        #self.get_logger().info("Elapsed Time: "+ str(self.elapsed_time))

        self.publisher.publish(msg)
        
        if(not self.turn_switch):
            self.elapsed_time = 0
            self.current_angle = 0
        else:
            self.elapsed_time += curr_time - self.last_time
            self.current_angle += angular_speed*(self.elapsed_time)
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