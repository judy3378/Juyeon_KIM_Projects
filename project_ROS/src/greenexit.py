#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author : Juyeon Kim
# Tested

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
import numpy as np

bridge = CvBridge()

#Ci après on lit les valeurs passees en parametres par le fichier launch
#Ils correspondent aux valeurs HSV des couleurs que l'on recherche

#Color 1 HSV values
lh0 = rospy.get_param("/lh0")
ls0 = rospy.get_param("/ls0")
lv0 = rospy.get_param("/lv0")

hh0 = rospy.get_param("/hh0")
hs0 = rospy.get_param("/hs0")
hv0 = rospy.get_param("/hv0")



#En fonction du fichier launch, on sait si on est dans la realite ou la simulation
#On adapte le programme en fonction
####real_situation = rospy.get_param("/real_sit")
real_situation = 0
print("la situation real",real_situation)

lowcolor0 = np.array([lh0, ls0, lv0])
highcolor0 = np.array([hh0, hs0, hv0])


#Les biais permettent de garantir la direction du robot sur le parcours
if real_situation == 1:
    bias = 0
    one_line_right = 100
    one_line_left = 100

else :
    bias = 15
    one_line_right = 60
    one_line_left = 60


challenge_count = 0

class line_detector:
    def __init__(self):
        self.bridge = CvBridge()

        #On definit le subscriber
        if real_situation == 0 :
            self.sub = rospy.Subscriber("/camera/image", Image, self.callback)
        else :
            self.sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.callback)

        #On definit le publisher
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

    #Fonction pour restreindre la region d'observation des lignes
    def region(self, img):
        height, width, _ = img.shape
        triangle = np.array([
                            [(0, height), (0, 2*height//3), (width, 2*height//3), (width, height)]
                            ])
        
        mask = np.zeros_like(img)
        
        mask = cv2.fillPoly(mask, triangle, [255, 255, 255])
        mask = cv2.bitwise_and(img, mask)
        return mask

    def callback(self, image_msg):
        global challenge_count

        #On verifie que la conversion du message a bien ete faite
        try:
            if real_situation == 0 :
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            else :
                cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        
        except CvBridgeError as e:
            print(e)

        #On isole les lignes qui nous interessent (zone d'interet = rectangle defini par la fct region)
        if real_situation == 0:
            cv_isolated = self.region(cv_image)

        else :
            cv_isolated = cv_image.copy()
        
        cv2.imshow("Masque", cv_isolated)
        cv_image_hsv = cv2.cvtColor(cv_isolated, cv2.COLOR_BGR2HSV)

        #Masque de couleur
        #inRange permet de trouver une couleur dans la plage HSV donnee
        cv_image_lines_c0 = cv2.inRange(cv_image_hsv, lowcolor0, highcolor0)
        cv_c1_output = cv2.bitwise_and(cv_isolated, cv_isolated, mask = cv_image_lines_c0) #Bitwise AND permet de montrer le masque et sa couleur


        cv2.imshow("Detected", cv_c1_output)

        #Moment nous permet de trouver les coordonnees du centre du masque
        moment_c0 = cv2.moments(cv_image_lines_c0)
      
        img_copy = cv_image.copy()
        height, width, _ = img_copy.shape
        iSeeBlue = False

        #Ligne de separation : Non utilise
        #Nous n'avons pas eu le temps d'implementer un systeme qui nous permet de detecter le passage d'un challenge et de changer le mode
        #La ligne est bien detectee, mais cela ne va pas plus loin
        
        if moment_c0["m00"] > 0 :   #ligne de separation de challenge
            cX3 = int(moment_c0["m10"] / moment_c0["m00"])
            cY3 = int(moment_c0["m01"] / moment_c0["m00"])
            print(moment_c0["m00"])

            iSeeBlue = True
            #print(iSeeBlue)
            

            cv2.circle(img_copy, (cX3, cY3), 10, [0, 0, 255], -1)

        elif moment_c0["m00"] == 0.0 and iSeeBlue == True :
            iSeeBlue == False
            challenge_count += 1
            print(challenge_count)
        

        cv2.imshow("Centre", img_copy)
        cv2.waitKey(1)

        c_vel = Twist()
        

if __name__ == '__main__':

    detector = line_detector()
    rospy.init_node("Lane_Detection")

    try:

        rospy.spin()

    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
