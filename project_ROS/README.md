# ROS Project

### Context, goal and evaluation of the project

In this project, you will control a simulated and a real Turtlebot 3 burger mobile robot in a realistic challenging environment, so as to make it navigate from one starting position to a goal position with different tasks to solve on the path. The navigation should successively exploit:
* images obtained from a simulated/real camera to detect and follow some lines,
* a laser scan obtained from a simulated/real LDS to detected and avoid some obstacles,
* and finally both of them to navigate in a challenging environment where both sensors are required together.

Further information can be found in `Project Description'

### Organization

For Challenge 1 : line following, the main idea is to compute the center of the lane and ensure the robot to move forward when it is in the center and adjust its orientation according to the center. To calculate the center of the two lines, yellow and white, masks are created to only detect and show these two colored lines, using color filters of cv2. A condition is applied if the robot can only see one of the two lines (red or white), where it moves along the detected line.
The code is under src file : `src/lane_following.py`


### Simulation

ROS and Gazebo are required to run the simulations.

To run the simulation for :
 * challenge 1 : run in terminal `roslaunch project challenge1.launch`
 * challenge 2 : `roslaunch project challenge2.launch`




