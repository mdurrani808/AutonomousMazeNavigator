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


        # MODULAR FLAGS
        self.gazebo = False
        self.finishable = True


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
            self.angle_range = 10
            self.FRONT_WALL_DIST = 0.35

            self.MIN_RIGHT_WALL_DIST = 0.20
            self.MAX_RIGHT_WALL_DIST = 0.40

            self.RIGHT_TURN_THRESHOLD = 0.50

            self.FRONT = 360
            self.LEFT = 540
            self.RIGHT = 180
            self.THETA = 10
            self.parallelBuffer = 0.01
            self.RANGE_MIN_IND = 0
            self.RANGE_MAX_IND = 720

            self.FRONT_RIGHT = 330

            self.RIGHT_FRONT = 200 # was 285
            self.RIGHT_BACK = 160

            self.parallelizing_error = 0.008
            self.RIGHT_SMALLER_PAR_RANGE = 10
            self.CURR_PAR_RANGE = 10
            self.RIGHT_TURN_SIGNAL_ANGLE = 290 # was 330
            self.RIGHT_TURN_BUFFER = 0.3
            self.parallelizing_direction = 0

            self.RIGHT_TURN_STRAIGHT_COUNT = 15
        # !--------------------------! #

        
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 100)
        self.pose_sub = self.create_subscription(LaserScan, "/scan", self.update_ranges, 100)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def has_front(self, front_dist, threshold_dist = None):
        if not threshold_dist:
            threshold_dist = self.FRONT_WALL_DIST
        
        return front_dist < threshold_dist
    
    def mean(self, arr):
        arr = [i for i in arr if i <= 1.5]
        if len(arr) > 0:
            return sum(arr) / len(arr)
        else:
            return math.inf
    
    def update_ranges(self, scan_msg):
        self.counter += 1
        rangesArr = scan_msg.ranges
        self.ranges = rangesArr

        front = self.get_dist_at_angle(self.FRONT, range_size=15)
        left = self.get_dist_at_angle(self.LEFT)
        right = self.get_dist_at_angle(self.RIGHT)
        right_front_dist = self.get_dist_at_angle(self.RIGHT_FRONT, range_size=5)
        right_back_dist = self.get_dist_at_angle(self.RIGHT_BACK, range_size=5)
        parallel_error = right_front_dist - right_back_dist
        right_signal_dist = self.get_dist_at_angle(self.RIGHT_TURN_SIGNAL_ANGLE)
        front_right = self.get_dist_at_angle(self.FRONT_RIGHT, range_size=3)
        

        
        if self.state == Motion_State.SPINNING:
            return
        
        if not self.hugging_right_wall:
            # if we are not at a wall, go straight until we find a wall,
            # then turn right.
            # if we are not sufficiently close to the wall, go straight
            print("not hugging wall. front dist: " + str(front))
            print("right_signal_dist " + str(right_signal_dist))
            print("normal right: " + str(right))
            print(1.5*self.FRONT_WALL_DIST)

            if not self.has_front(front):
                self.current_action = Actions.STRAIGHT
                self.state = Motion_State.STRAIGHT
            else:
                self.execute_action(Actions.LEFT_TURN)
                self.hugging_right_wall = True

        # IF WE ARE CURRENTLY MOVING STRAIGHT
        elif self.current_action == Actions.STRAIGHT:
            # once we know we are hugging the right wall and going straight, keep parallel to
            # the right wall.
            self.state = Motion_State.STRAIGHT

            # if our right_front distance ever becomes greater than front_wall_dist + some buffer,
            # we want to parallelize right so that we go closer to the wall, then continue parallelizing

            print("right distance: " + str(right))
            print("right_front_dist: " + str(right_front_dist))

            if right_front_dist > self.MAX_RIGHT_WALL_DIST:
                print("moving to right wall!")
                self.parallelizing_direction = 1

            elif right_front_dist < self.MIN_RIGHT_WALL_DIST:
                print("moving away from right wall!")
                self.parallelizing_direction = -0.5

            # this will cover the normal parallelizing case, where we are next to a wall, but not too far
            elif abs(parallel_error) > self.parallelBuffer:
                # if the parallelizing error is already large, we don't want to magnify it
                # too much.
                if parallel_error <= 1:
                    self.parallelizing_direction = 10*parallel_error
                else:
                    self.parallelizing_direction = 3*parallel_error
            else:
                self.parallelizing_direction = 0

            

            # if we have exited the maze, go to spinning state
            if left == math.inf and right == math.inf:
                self.current_action = Actions.SPIN
                self.state = Motion_State.SPINNING

            # if we have an open right side, we should prioritize turning right.
            elif right_front_dist > self.RIGHT_TURN_THRESHOLD:
                self.execute_action(Actions.RIGHT_TURN)


            # otherwise, if there is a wall ahead of us, turn left.
            elif self.has_front(front) or self.has_front(front_right):
                self.execute_action(Actions.LEFT_TURN)


        # IF WE ARE CURRENTLY TURNING LEFT
        elif self.current_action == Actions.LEFT_TURN:
            # keep turning left until we are parallel to right wall.
            # if right_front and right_back are equal, but right is longer than both,
            # then we are turning a corner. So, keep turning.
            self.state = Motion_State.TURNING_LEFT
            if abs(parallel_error) < self.parallelBuffer and right < right_front_dist and self.counter > 5:
                self.execute_action(Actions.STRAIGHT)


        # IF WE ARE CURRENTLY TURNING RIGHT
        elif self.current_action == Actions.RIGHT_TURN:

            # turn until we have a wall in front at approx 45 degrees
            self.state = Motion_State.TURNING_RIGHT

            if self.counter < self.RIGHT_TURN_STRAIGHT_COUNT and not self.turned and not self.has_front(front):
                print("counter straight")
                self.state = Motion_State.STRAIGHT_NO_PARALLELIZING
                self.front_target = right
                if front < 1.5*self.FRONT_WALL_DIST:
                    self.cleared_front_wall = False

            # next turn until we get a wall reading at the given "signal" distance
            # also check that we haven't finished partial turning already

            # if we had a front wall at the beginning of the turn, we must check to see that
            # we have turned past the wall before stopping our turn
            elif ((right_signal_dist > 1.5*self.FRONT_WALL_DIST or not self.cleared_front_wall) 
                  and not self.partial_turned):
                print("turning")
                self.state = Motion_State.TURNING_RIGHT
                self.turned = True
                # once we have turned so that the front and right no longer point to a wall,
                # set the cleared_front_wall variable to true
                if not self.cleared_front_wall and front >= 1.5*self.FRONT_WALL_DIST:
                    self.cleared_front_wall = True

            # next, move straight until we can parallelize the bot to the wall. Also, set the signal
            # for partial turning to true, so we do not try to partial turn again.
            elif (right > 1.5*self.FRONT_WALL_DIST) and self.cleared_front_wall:
                self.partial_turned = True
                self.state = Motion_State.STRAIGHT_NO_PARALLELIZING
                self.counter = 0

            else:
                #print("parallelizing!!\n\n\n----------------")
                self.execute_action(Actions.STRAIGHT)
            
        # WE ARE CURRENTLY PARALLELIZING TO A RIGHT WALL
        elif self.current_action == Actions.PARALLELIZE:
            # use a closer set of 2 lines to parallelize.
            # assumes we will not be in a corner situation
            # we use a closer set since if we have just made a right turn
            # to trace a thin wall, while the right turn action guarantees
            # right_back is at this wall before starting this action,
            # we don't necessarily know the same about right_front, so
            # we take a set of closer lines to parallelize off of, to be safe

            if self.counter <= 1:
                self.CURR_PAR_RANGE = self.RIGHT_SMALLER_PAR_RANGE

            
            front_angle = self.RIGHT + self.CURR_PAR_RANGE
            back_angle = self.RIGHT - self.CURR_PAR_RANGE

            front_line = self.get_dist_at_angle(front_angle, 1)
            back_line = self.get_dist_at_angle(back_angle, 1)

            #if front_line > 1.5*self.MAX_RIGHT_WALL_DIST or back_line > 1.5*self.MAX_RIGHT_WALL_DIST and self.CURR_PAR_RANGE > 1:
                #self.CURR_PAR_RANGE -= 1

            smaller_par_err = front_line - back_line
            print("front_line angle: " + str(front_angle) + ", front_line dist: " + str(front_line))
            print("back_line angle: " + str(back_angle) + ", back_line dist: " + str(back_line))
            print(smaller_par_err)
            print("----------------------------------------\n")

            if abs(smaller_par_err) > self.parallelizing_error:
                self.state = Motion_State.PARALLELIZING
                self.parallelizing_direction = 10*smaller_par_err
            else:
                self.execute_action(Actions.STRAIGHT)

        elif self.current_action == Actions.SPIN:
            if self.counter <= 10:
                self.state = Motion_State.STRAIGHT

            else:
                self.state = Motion_State.SPINNING

            

        
        front_half = self.get_angle_range(360, range_size=180)
        front_half = [i for i in front_half if i != math.inf]
        print("number of non-infinite values: " + str(len(front_half)))
        if len(front_half) < 270 and self.current_action != Actions.SPIN and self.finishable:
            self.execute_action(Actions.SPIN)
            
    def execute_action(self, action):
        self.counter = 0

        if action == Actions.LEFT_TURN:
            self.get_logger().info("turn left!")
            self.current_action = Actions.LEFT_TURN
            self.state = Motion_State.TURNING_LEFT

        elif action == Actions.RIGHT_TURN:
            self.get_logger().info("turn right!")
            self.current_action = Actions.RIGHT_TURN
            self.state = Motion_State.TURNING_RIGHT
            self.partial_turned = False
            self.turned = False

        elif action == Actions.STRAIGHT:
            self.get_logger().info("go straight!")
            self.parallelizing_direction = 0
            self.current_action = Actions.STRAIGHT
            self.state = Motion_State.STRAIGHT

        elif action == Actions.PARALLELIZE:
            self.get_logger().info("parallelize!")
            self.current_action = Actions.PARALLELIZE
            self.state = Motion_State.PARALLELIZING

        elif action == Actions.SPIN:
            self.get_logger().info("Finished!")
            self.current_action = Actions.SPIN
            self.state = Motion_State.STRAIGHT

   
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
        ANGULAR = .5
        msg = Twist()
                
        if self.state == Motion_State.STRAIGHT:
            msg.linear.x = LINEAR
            msg.angular.z = self.parallelizing_direction * (-ANGULAR)
        elif self.state == Motion_State.TURNING_LEFT:
            msg.linear.x = 0.0
            msg.angular.z = ANGULAR
        elif self.state == Motion_State.TURNING_RIGHT:
            msg.linear.x = 0.0
            msg.angular.z = -ANGULAR
        elif self.state == Motion_State.STOPPED:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        elif self.state == Motion_State.STRAIGHT_NO_PARALLELIZING:
            msg.linear.x = LINEAR
        elif self.state == Motion_State.SPINNING:
            msg.angular.z = ANGULAR * 4
        elif self.state == Motion_State.PARALLELIZING:
            msg.angular.z = self.parallelizing_direction * (-ANGULAR)
        
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