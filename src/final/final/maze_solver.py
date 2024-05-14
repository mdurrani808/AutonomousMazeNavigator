import rclpy
from rclpy.node import Node
from enum import Enum
import rclpy.time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class States(Enum):
    TURN_RIGHT: int = 1,
    STRAIGHT: int = 2,
    TURN_LEFT: int = 3,
    PARALLELIZE: int = 4,
    UNKNOWN: int = 5,

class WallFollowerVFinal(Node):
    def __init__(self):
        super().__init__('WallFollowerVFinal')
        # sets up all the necessary publishers and subscribers for the lidar topic and velocity topic
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 100)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.update_ranges, 10)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        
        self.gazebo = False
        self.tuning = False
        self.enable_logging = False
        self.got_first_message = False
        self.has_started = False # starting is defined as seeing our first wall in front of us. 
        #sets the 1/2 the angle we use to calculate a certain distance (ex. theta = 10 means we will use the average of a 20 degree range to calculate the angle)
        self.theta = int(10)
        self.desired_turn_time = 4.75
        self.parallelizing_error = 0.0
        self.parallelBuffer = 0.001
        self.kp = 50.0
        
        self.elapsed_time = 0.0
        self.last_time = 0.0
        # this defines the center of all the ranges that we care about from the lidar scan
        if(self.gazebo):
            self.front = 0
            self.right_orth = 270
            self.right_front = 315
            self.right_back = 245
            
            # angles used while parallelizing
            self.right_front_parallel_angle = 255
            self.right_back_parallel_angle = 285
            
            self.left_front_parallel_angle = 105
            self.left_back_parallel_angle = 75
            self.parallel_theta = int(5.0)
            
            # This sets the distance we use to check whether or not there is "something" within that range
            self.min_front_dist = 0.70
            self.min_right_orth_dist = 0.6
            self.min_right_front_dist = 0.6
            self.min_right_back_dist = 1.05
            
            self.linear = .5
            self.angular = 0.5
            self.parallel_angular = 0.075
        else:
            self.front = 360
            self.right_orth = 180
            self.right_front = 270
            self.right_back = 150
            
            # angles used while parallelizing
            self.right_front_parallel_angle = 160
            self.right_back_parallel_angle = 200
            
            self.left_front_parallel_angle = 560
            self.left_back_parallel_angle = 520
            self.parallel_theta = int(1.0)
            
            # This sets the distance we use to check whether or not there is "something" within that range
            self.min_front_dist = .5
            self.min_right_orth_dist = 0.6
            self.min_right_front_dist = 0.5
            self.min_right_back_dist = 1.05
            
            self.linear = .5
            self.angular = 0.5
            self.parallel_angular = 0.07

        # stores the relevant current distances that are computed whenever we get a new scan_message from the lidar.
        self.curr_front_dist = 0.0
        self.curr_right_orth_dist = 0.0
        self.curr_right_front_dist = 0.0
        self.curr_right_back_dist = 0.0

        self.state = None
        self.last_state = None
        self.last_turn_state = None
        
        # Logging
        self.var_has_front = False
        self.var_has_right_orth = False
        self.var_has_right_front = False
        self.var_has_right_back = False
        
        self.last_has_front = False
        self.last_has_right_orth = False
        self.last_has_right_front = False
        self.last_has_right_back = False

    def update_ranges(self, scan_msg):
        self.got_first_message = True
        lidar_data = scan_msg.ranges
        # calculate all of the relevant distances using the scan_msg from the lidar
        self.curr_front_dist = lidar_data[self.front]
        self.curr_right_orth_dist = self.mean(lidar_data[self.right_orth - self.theta : self.right_orth + self.theta])
        self.curr_right_front_dist = self.mean(lidar_data[self.right_front - self.theta : self.right_front + self.theta])
        self.curr_right_back_dist = self.mean(lidar_data[self.right_back - self.theta : self.right_back + self.theta])
        
        # this data is only used when parallelizing after turning
        right_front_parallel = self.mean(lidar_data[(self.right_front_parallel_angle - self.parallel_theta) : (self.right_front_parallel_angle + self.parallel_theta)])
        right_back_parallel = self.mean(lidar_data[self.right_back_parallel_angle - self.parallel_theta:self.right_back_parallel_angle + self.parallel_theta])
        
        left_front_parallel = self.mean(lidar_data[self.left_front_parallel_angle - self.parallel_theta:self.left_front_parallel_angle + self.parallel_theta])
        left_back_parallel = self.mean(lidar_data[self.left_back_parallel_angle - self.parallel_theta:self.left_back_parallel_angle + self.parallel_theta])
        
        
        #cache the last state (to allow us to check for state transitions)
        self.last_state = self.state
        
        # set up the statemachine for following the right hand rule for maze following.
        # The right hand rule for maze has you keep your right hand on the maze wall, guarenteeing that you will reach the end of the maze in simple conditions
        
        # see the state diagram found within the report for further explanation
        
        # start by caching allthe has_front values (we need this to log state transitions)
        self.last_has_front = self.var_has_front
        self.last_has_right_front = self.var_has_right_front
        self.last_has_right_orth = self.var_has_right_orth
        self.last_has_right_back = self.var_has_right_back
        
        self.var_has_front = self.has_front()
        self.var_has_right_front = self.has_right_front()
        self.var_has_right_orth = self.has_right_orth()
        self.var_has_right_back = self.has_right_back()
        if self.has_started:
            
            if(self.state != States.TURN_LEFT and self.state != States.TURN_RIGHT):
                self.elapsed_time = 0
                # if we are parallelizing, calculate the error and then check if we are done yet.
                                # We just need to continue against a wall if the wall does exist. 
                #self.print(str((not self.var_has_front and not self.var_has_right_back) or (not self.var_has_front and self.var_has_right_front) or (not self.var_has_front and self.var_has_right_orth)))
                if self.state == States.PARALLELIZE:
                    
                    # if we just turned left or the left wall is too far away, we parallelize off the right wall.
                    if (self.last_turn_state == States.TURN_LEFT) or left_front_parallel > right_front_parallel:
                        self.parallelizing_error = right_front_parallel-right_back_parallel
                    
                    # otherwise, use the left wall (the order is flipped so the sign matches that of the right error as needed)
                    else:
                        self.parallelizing_error = left_back_parallel-left_front_parallel

                    if abs(self.parallelizing_error) < self.parallelBuffer:
                        self.state = States.STRAIGHT
                        self.get_logger().info(f"Transitioning from {self.last_state} to {self.state} because parallel")
                
                # this means are are at a corner, we need to turn left to follow the wall
                elif(self.var_has_front and self.var_has_right_orth):
                    self.state = States.TURN_LEFT
                    self.last_turn_state = States.TURN_LEFT
                
                elif((not self.var_has_front and not self.var_has_right_back) or (not self.var_has_front and self.var_has_right_front) or (not self.var_has_front and self.var_has_right_orth)):    
                    self.state = States.STRAIGHT
                    
                # if there is soemthing in the right back (we have passed a wall) and nothing in the front, we need to turn right to follow the outside corner
                elif(self.var_has_right_back and not (self.var_has_right_orth or self.var_has_right_front) and self.curr_right_orth_dist > self.curr_right_back_dist and self.curr_right_back_dist > .55):
                    self.state = States.TURN_RIGHT
                    self.last_turn_state = States.TURN_RIGHT
                # something went wrong and we don't know what state we are in! fuck it go right
                else:
                    self.state = States.STRAIGHT
        else:
            if self.has_front():
                self.has_started = True
                self.state = States.TURN_LEFT
                self.last_state = States.STRAIGHT
                self.last_turn_state = States.TURN_LEFT
    
    def timer_callback(self):
        LINEAR = self.linear
        ANGULAR = self.angular
        msg = Twist()
        # print logging messages on every state transition or variable change
        if(self.enable_logging and self.got_first_message):
            if(self.last_state != self.state):
                self.print(f"{self.last_state} to {self.state}")
                self.last_state = self.state
            if(self.last_has_front != self.var_has_front):
                self.print_has_side()
                self.last_has_front = self.var_has_front
            if(self.last_has_right_orth != self.var_has_right_orth):
                self.print_has_side()
                self.last_has_right_orth = self.var_has_right_orth
            if(self.last_has_right_front != self.var_has_right_front):
                self.print_has_side()
                self.last_has_right_front = self.var_has_right_front
            if(self.last_has_right_back != self.var_has_right_back):
                self.print_has_side()
                self.last_has_right_back = self.var_has_right_back
        
        # if we are tuning our state transitions (ex. with the teleop key) we don't want it to be moving on its own
        if(not self.tuning):
        #if the state just changed to turning, start the timer
            if self.last_state == States.STRAIGHT and (self.state == States.TURN_LEFT or self.state == States.TURN_RIGHT):
                self.elapsed_time = 0
                self.print("START TURNING")
            
            # if the are turning and the timer has passed, start parallelizing
            if (self.state == States.TURN_LEFT or self.state == States.TURN_RIGHT) and self.elapsed_time >= self.desired_turn_time:
                self.state = States.PARALLELIZE
                self.print("PARALLELIZING")
            
            if self.state == States.UNKNOWN:
                self.state = self.last_state
            
            # go straight at the start or if we state is straight, go straight
            if not self.has_started or self.state == States.STRAIGHT:
                msg.linear.x = LINEAR
                msg.angular.z = 0.0
            elif self.state == States.TURN_LEFT:
                msg.angular.z = ANGULAR
            elif self.state == States.TURN_RIGHT:
                msg.angular.z = -ANGULAR
            elif self.state == States.PARALLELIZE:
                msg.angular.z = self.parallel_angular * self.sign(self.parallelizing_error) * abs(self.parallelizing_error * self.kp)

            self.publisher.publish(msg)
            
            # onlt start the timer once the rover has started the maze
            if(self.has_started):
                curr_time = self.get_clock().now().seconds_nanoseconds()
                
                curr_time = curr_time[0] + curr_time[1] / 1000000000.0
                
                # sometimes we get a really large value out of this, not sure why
                if(curr_time - self.last_time < 1):
                    self.elapsed_time += curr_time - self.last_time
                self.last_time = curr_time
            self.last_state = self.state
            
    def sign(self,num):
        return -1 if num < 0 else 1
    
    def stopTurtleBot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        # set all the necessary code for turning and everything 
    def has_front(self):
        return self.curr_front_dist < self.min_front_dist
    
    def has_right_orth(self):
        return self.curr_right_orth_dist < self.min_right_orth_dist
    
    def has_right_front(self):
        return self.curr_right_front_dist < self.min_right_front_dist
    
    def has_right_back(self):
        return self.curr_right_back_dist < self.min_right_back_dist
    
    # wrapper for the ROS logger
    def print(self, string):
        self.get_logger().info(string)
        
    
    def print_has_side(self):
        self.print(f"Has Front: {str(self.has_front())} and {str(self.curr_front_dist)}")
        self.print(f"Has Right Orth: {str(self.has_right_orth())} and {str(self.curr_right_orth_dist)}")
        self.print(f"Has Right Front: {str(self.has_right_front())} and {str(self.curr_right_front_dist)}")
        self.print(f"Has Right Back: {str(self.has_right_back())} and {str(self.curr_right_back_dist)}")
        self.print(f"--------------------")
        
    # mean function that accounts for infinite values (which is returned when there is nothing in the detected range of the lidar.)
    def mean(self, arr):
        arr = [i for i in arr if i != math.inf]
        if len(arr) > 0:
            return sum(arr) / len(arr)
        else:
            return math.inf
    
    
def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerVFinal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()