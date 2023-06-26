# ROS Project

### Context

The main objective of this project is to control a Turtlebot 3 burger mobile robot in a simulated and real environment. The goal of this project is to develop navigation skills and test the robot’s sensorimotor capabilities by making it navigate from a starting position to a goal position while solving different tasks on the path.
The navigation should successively exploit :
* images obtained from a simulated/real camera to detect and follow some lines (**Challenge 1**), 
* a laser scan obtained from a simulated/real LDS to detected and avoid some obstacles (**Challenge 2**),
* and finally both of them to navigate in a challenging environment where both sensors are required together (**Challenge 3**).


For Challenge 1: [lane following](/project_ROS/src/lane_following.py), the main idea is to compute the center of the lane and ensure the robot moves forward when it is in the center and adjusts its orientation according to the center. To calculate the center of the two lines, yellow and white, masks are created to only detect and show these two colored lines, using color filters of **cv2**. A condition is applied if the robot can only see one of the two lines (red or white), where it moves along the detected line.

For Challenge 2: Corridor, `src/Corridor_gazebo.py` for simulation under Gazebo, and `src/Corridor_robot.py` for real robot use.

Further information can be found in [Project_Description](/project_ROS/Project_Description.pdf) and [Project_Report](/project_ROS/Project_Report.pdf).


### Install and Run

[ROS Noetic](http://wiki.ros.org/ROS/Installation) and [Gazebo](https://gazebosim.org/docs) are required to run the simulations.

To run the simulation for :
 * Challenge 1: run in terminal `roslaunch project challenge1.launch`
 * Challenge 2: `roslaunch project challenge2.launch`




