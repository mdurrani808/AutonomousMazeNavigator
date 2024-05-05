import rclpy
from rclpy.node import Node
from enum import Enum
import rclpy.time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class TestingStates(Enum):
    TURN_RIGHT: int = 1,
    STRAIGHT: int = 2,
    TURN_LEFT: int = 3,
    UNKNOWN: int = 4,

class WallFollowerVFinal(Node):
    def __init__(self):
        super().__init__('WallFollowerVFianl')
        # sets up all the necessary publishers and subscribers for the lidar topic and velocity topic
        #self.publisher = self.create_publisher(Twist, "/cmd_vel", 100)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.update_ranges, 10)
        self.timer = self.create_timer(0.25, self.timer_callback)
        
        
        self.gazebo = True
        self.enable_logging = True
        self.got_first_message = False
        #sets the 1/2 the angle we use to calculate a certain distance (ex. theta = 10 means we will use the average of a 20 degree range to calculate the angle)
        self.theta = int(10)
        
        # this defines the center of all the ranges that we care about from the lidar scan
        if(self.gazebo):
            self.front = 0
            self.right_orth = 270
            self.right_front = 315
            self.right_back = 240
            
            # This sets the distance we use to check whether or not there is "something" within that range
            self.min_front_dist = .5
            self.min_right_orth_dist = .5
            self.min_right_front_dist = .5
            self.min_right_back_dist = 1.05
        else:
            self.front = 0
            self.right_orth = 270
            self.right_front = 315
            self.right_back = 225
            
            # This sets the distance we use to check whether or not there is "something" within that range
            self.min_front_dist = .5
            self.min_right_orth_dist = .5
            self.min_right_front_dist = .5
            self.min_right_back_dist = 1.05

        # stores the relevant current distances that are computed whenever we get a new scan_message from the lidar.
        self.curr_front_dist = 0.0
        self.curr_right_orth_dist = 0.0
        self.curr_right_front_dist = 0.0
        self.curr_right_back_dist = 0.0

        self.state = None
        self.last_state = None
        
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
        self.curr_front_dist = lidar_data[0]
        self.curr_right_orth_dist = self.mean(lidar_data[self.right_orth - self.theta : self.right_orth + self.theta])
        self.curr_right_front_dist = self.mean(lidar_data[self.right_front - self.theta : self.right_front + self.theta])
        self.curr_right_back_dist = self.mean(lidar_data[self.right_back - self.theta + 5 : self.right_back + self.theta +5])
        
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
        
        # this means are are at a corner, we need to turn left to follow the wall
        if(self.var_has_front and self.var_has_right_orth):
            self.state = TestingStates.TURN_LEFT
            
        # We just need to continue against a wall if the wall does exist. 
        elif((not self.var_has_front and not self.var_has_right_back) or (not self.var_has_front and self.var_has_right_front) or (not self.var_has_front and self.var_has_right_orth)):    
            self.state = TestingStates.STRAIGHT
            
        # if there is soemthing in the right back (we have passed a wall) and nothing in the front, we need to turn right to follow the outside corner
        elif(self.var_has_right_back and not (self.var_has_front or self.var_has_right_front) and self.curr_right_orth_dist > self.curr_right_back_dist and self.curr_right_back_dist > .5):
            self.state = TestingStates.TURN_RIGHT
        
        # something went wrong and we don't know what state we are in!
        else:
            if(not self.var_has_right_back and not self.var_has_front and not self.var_has_right_front and not self.var_has_right_orth):
                self.state = TestingStates.TURN_RIGHT
            else:
                self.state = self.last_state
    
    def timer_callback(self):
        #will log any state transitions (makes up for lack of realtime debugger)
        #self.print("RF " + str(self.curr_right_front_dist))
        #self.print("RO" + str(self.curr_right_orth_dist))
        #self.print("RB" + str(self.curr_right_back_dist))
        #self.print("--------")
        if(self.enable_logging and self.got_first_message):
            if(self.last_state != self.state):
                self.print(f"{self.last_state} to {self.state}")
                self.last_state = self.state
                
            if(self.last_has_front != self.var_has_front):
                self.print(f"Has Front: {str(self.has_front())} and {str(self.curr_front_dist)}")
                self.print(f"Has Right Orth: {str(self.has_right_orth())} and {str(self.curr_right_orth_dist)}")
                self.print(f"Has Right Front: {str(self.has_right_front())} and {str(self.curr_right_front_dist)}")
                self.print(f"Has Right Back: {str(self.has_right_back())} and {str(self.curr_right_back_dist)}")
                self.print(f"--------------------")
                self.last_has_front = self.var_has_front
                
            if(self.last_has_right_orth != self.var_has_right_orth):
                self.print(f"Has Front: {str(self.has_front())} and {str(self.curr_front_dist)}")
                self.print(f"Has Right Orth: {str(self.has_right_orth())} and {str(self.curr_right_orth_dist)}")
                self.print(f"Has Right Front: {str(self.has_right_front())} and {str(self.curr_right_front_dist)}")
                self.print(f"Has Right Back: {str(self.has_right_back())} and {str(self.curr_right_back_dist)}")
                self.print(f"--------------------")
                self.last_has_right_orth = self.var_has_right_orth
                
            if(self.last_has_right_front != self.var_has_right_front):
                self.print(f"Has Front: {str(self.has_front())} and {str(self.curr_front_dist)}")
                self.print(f"Has Right Orth: {str(self.has_right_orth())} and {str(self.curr_right_orth_dist)}")
                self.print(f"Has Right Front: {str(self.has_right_front())} and {str(self.curr_right_front_dist)}")
                self.print(f"Has Right Back: {str(self.has_right_back())} and {str(self.curr_right_back_dist)}")
                self.print(f"--------------------")
                self.last_has_right_front = self.var_has_right_front
                
            if(self.last_has_right_back != self.var_has_right_back):
                self.print(f"Has Front: {str(self.has_front())} and {str(self.curr_front_dist)}")
                self.print(f"Has Right Orth: {str(self.has_right_orth())} and {str(self.curr_right_orth_dist)}")
                self.print(f"Has Right Front: {str(self.has_right_front())} and {str(self.curr_right_front_dist)}")
                self.print(f"Has Right Back: {str(self.has_right_back())} and {str(self.curr_right_back_dist)}")
                self.print(f"--------------------")
                self.last_has_right_back = self.var_has_right_back
        
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