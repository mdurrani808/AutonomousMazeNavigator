Group: Mohammad Durrani and Karthik Taranath
We completed part 1.1, 1.2, 1.3 (bonus), and 2.
1.1 Video: https://drive.google.com/file/d/1nuX4SP4A1tmJ1h6N8xvXTaZpFQBhzAiS/view?usp=sharing
1.2 Video: https://drive.google.com/file/d/1D0XGNKMTcVeh2yBp3wLEOtvOh9rkXBkK/view?usp=sharing

In the 1.1 video, the video cuts off. This is due to windows and the VM crashing due to lack of resources. The assignment is otherwise complete.

1.3 SLAM:
The robot starts out with no information about its surroundings. It starts collecting data by initializing its location in a pose graph, which is represented as a node. A corresponding LiDAR scan is added to the map in the pose graph. Slowly, the robot starts moving around its environment, creating nodes from LiDAR scans around the environment. Odometry is also used to keep track of the robot’s location. Nodes are created until the robot creates a full loop, at which point, Pose Graph Optimization occurs. By using distinct features in the environment to confirm that the robot is in the same place that it started the loop in, the error between the real and estimated locations of the robot is used to calculate the error of the odometer along its entire path. This way, the error can be applied to each node in the pose graph, and the resulting, adjusted map should be an accurate representation of the robot’s environment.

2: Code is attached. Distance was increased from .5 to 1 to ensure that we would not accidentally hit a wall, but is logically equivalent. 