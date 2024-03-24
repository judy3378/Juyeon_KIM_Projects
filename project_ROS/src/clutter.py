#!/usr/bin/env python3
# -*- coding: utf-8 -
# Author : Juyeon Kim
# Tested

import rospy
from geometry_msgs.msg import Twist, Pose2D, Point
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from cv_bridge import CvBridge,  CvBridgeError
import cv2
import numpy as np

class NavigationChallenge:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('clutter_challenge', anonymous=True)

        # Initialize subscribers
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        
        # Create a publisher for the target point
        self.target_pub = rospy.Publisher('target_point', Point, queue_size=10)

        # Initialize publishers
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Create a CvBridge object to convert ROS messages to OpenCV images
        self.bridge = CvBridge()
        
        # Initialize variables
        self.obstacle_detected = False
        self.exit_detected = False
        self.exit_zone = [(5, 5), (-5, 5), (-5, -5), (5, -5)] # Coordinates of the exit zone
        self.current_pose = Pose2D() # Current position and orientation of the robot
        self.scan_range = 0.5 # Maximum range for laser scan
        self.linear_speed = 0.05 # Linear speed for robot movement
        self.angular_speed = 0.02 # Angular speed for robot movement
        self.stop_distance = 0.2 # Distance to obstacle at which robot should stop moving
        self.navigating = False # Flag indicating whether the robot is currently navigating or not

    def laser_callback(self, msg):
        # Check for obstacles within scan range
        for distance in msg.ranges:
            if distance < self.scan_range:
                self.obstacle_detected = True
                return
        self.obstacle_detected = False

    def image_callback(self, msg):
        # TODO: Implement exit detection using camera
        # Convert image message to usable format
        
        #On verifie que la conversion du message a bien ete faite
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
        except CvBridgeError as e:
            print(e)
            
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv2.imshow("Masque", cv_image)
        
        
        # Apply color thresholding to extract green pixels
        cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([100, 255, 255])
        lower_blue= np.array([240, 100, 100])
        upper_blue = np.array([300, 255, 255])
        
        cv_images_lines_c1 = cv2.inRange(cv_image_hsv, lower_green, upper_green)
        cv_c1_output = cv2.bitwise_and(cv_image, cv_image, mask=cv_images_lines_c1)

        cv_images_lines_c2 = cv2.inRange(cv_image_hsv, lower_blue, upper_blue)
        cv_c2_output = cv2.bitwise_and(cv_image, cv_image, mask=cv_images_lines_c2)
        
        cv_output_lines = cv2.bitwise_or(cv_c1_output, cv_c2_output) #On additionne le tout pour afficher toutes les lignes detectees

        #cv_image_lines_c2 = cv2.inRange(cv_image_hsv, )
        cv2.imshow("Detected", cv_output_lines)
        # Find contours of green pixels
        contours, _ = cv2.findContours(cv_images_lines_c1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # If at least two contours are found, assume they are the exit posts
        if len(contours) >= 2:
            # Calculate centroids of contours
            centroids = []
            for cnt in contours:
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    centroids.append((cx, cy))

            # Sort centroids from left to right
            centroids.sort()

            # Calculate target position as midpoint between centroids
            if len(centroids) >= 2:
                exit_x = int((centroids[0][0] + centroids[1][0]) / 2)
                exit_y = int((centroids[0][1] + centroids[1][1]) / 2)
            else:
                # Handle the case where there are not enough centroids
                exit_x = 0
                exit_y=0

            # Publish target position as a Point message
            target_msg = Point()
            target_msg.x = exit_x
            target_msg.y = exit_y
            print("target x {}, y{}".format(target_msg.x, target_msg.y) )

            self.target_pub.publish(target_msg)


    def navigate(self):
        # Navigate through the cluttered environment
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            if not self.navigating:
                # Stop the robot if it's not currently navigating
                self.stop_robot()
                rate.sleep()
                continue
            
            if self.obstacle_detected:
                # Obstacle detected, turn around
                self.turn_around()
            else:
                # No obstacle detected, move forward
                self.move_forward()

            if self.exit_detected:
                # Exit detected, stop navigating
                self.stop_robot()
                break

            rate.sleep()

    def stop_robot(self):
        # Stop the robot's movement
        self.vel_pub.publish(Twist())

    def move_forward(self):
        # Move the robot forward
        vel_msg = Twist()
        vel_msg.linear.x = self.linear_speed
        self.vel_pub.publish(vel_msg)

    def turn_around(self):
        # Turn the robot around
        vel_msg = Twist()
        vel_msg.angular.z = self.angular_speed
        self.vel_pub.publish(vel_msg)

    def start_navigation(self):
        # Start the robot's navigation
        self.navigating = True

    def stop_navigation(self):
        # Stop the robot's navigation
        self.navigating = False

    def get_current_pose(self):
        # Get the current position and orientation of the robot
        return self.current_pose

    def set_current_pose(self, x, y, theta):
        # Set the current position and orientation of the robot
        self.current_pose.x = x
        self.current_pose.y = y
        self.current_pose.theta = theta

    def is_in_exit_zone(self, x, y):
        # Check if the given coordinates are within the exit zone
        return (x, y) in self.exit_zone
    
if __name__ == '__main__':
    nc = NavigationChallenge()
    # Initialize ROS node
    rospy.init_node('clutter_challenge', anonymous=True)
    nc.set_current_pose(0, 0, 0) # Set robot's starting position
    nc.start_navigation() # Start robot's navigation
    while not nc.is_in_exit_zone(nc.get_current_pose().x, nc.get_current_pose().y):
        continue
    nc.stop_navigation() # Stop robot's navigation
    try:
        
        
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass

