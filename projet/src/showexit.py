c
import rospy
from geometry_msgs.msg import Point, Twist
import cv2
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import numpy as np

class ImageNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('image_node', anonymous=False)

        # Subscribe to the camera image topic
        self.image_sub = rospy.Subscriber('camera/image', Image, self.image_callback)

        # Create a publisher for the target point
        self.target_pub = rospy.Publisher('target_point', Point, queue_size=10)

        # Create a publisher for the robot velocity
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Create a CvBridge object to convert ROS messages to OpenCV images
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert image message to usable format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Mask", cv_image)
        # Apply color thresholding to extract green pixels
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([90, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        cv_c1_output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        cv2.imshow("cv_c1_output", cv_c1_output)
        cv2.imshow("mask", mask)
        
        
        #cv2.destroyAllWindows()
    
        # Find contours of green pixels
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
            self.target_pub.publish(target_msg)

if __name__ == '__main__':
    print("trynode")
    rospy.init_node('image_node', anonymous=False)
    try:
        
        rospy.spin()
        print("spin")
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
