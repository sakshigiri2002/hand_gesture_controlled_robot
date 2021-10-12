#!/usr/bin/env python

#Imorting Important Libraries
import rospy
from geometry_msgs.msg import Twist
from turtlebot3_msgs import *
import time
from std_srvs.srv import Empty
import numpy as np
import cv2
import math

#Counters to check the angles

cnt2 = 0
cnt3 = 0
cnt4 = 0
cnt5 = 0
cnt6 = 0
#global vel_msg
#Capturing The Video 

cap = cv2.VideoCapture(0)

while(cap.isOpened()):
    
    ret , img = cap.read()

    #Cropping the image

    cv2.rectangle(img,(0,0),(300,300),(0,255,0),0)
    crop_img = img[0:300,0:300]

    #Converting the img frame into gray scale 
    gray = cv2.cvtColor(crop_img,cv2.COLOR_RGB2GRAY)
    blur1 = cv2.GaussianBlur(gray,(5,5),0)

    #Finging the thresholding for frame
    ret,thresh = cv2.threshold(blur1,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

    #Applying Dilation and erosion for making the thresholding image proper
    kernel = np.ones((3,3),dtype = "uint8")
    dilation = cv2.dilate(thresh,kernel,iterations=3)
    erosion = cv2.erode(dilation,kernel,iterations=2)
    blur2 = cv2.GaussianBlur(erosion,(5,5),0)

    #Finding Contours of image 
    contours , hierarchy = cv2.findContours(blur2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    contours = max(contours ,key = lambda x: cv2.contourArea(x))
    cv2.drawContours(crop_img,[contours],-1,(255,255,0),2)

    #Finding convex Hull
    hull = cv2.convexHull(contours)
    cv2.drawContours(img,[hull],-1,(0,255,255),2)

    #Finding the convexity defects
    hull = cv2.convexHull(contours,returnPoints=False)
    defects = cv2.convexityDefects(contours,hull)

    if defects is not None:
        cnt = 0

    for i in range(defects.shape[0]):
        s,e,f,d = defects[i,0]
        start = tuple(contours[s][0])
        end = tuple(contours[e][0])
        far = tuple(contours[f][0])

        bottommost = tuple(contours[contours[:,:,1].argmax()][0])
        rightmost = tuple(contours[contours[:,:,0].argmax()][0])
        leftmost = tuple(contours[contours[:,:,0].argmin()][0])
        topmost = tuple(contours[contours[:,:,1].argmin()][0])

        cv2.circle(crop_img,bottommost,5,[255,0,0],-1)
        cv2.circle(crop_img,rightmost,5,[255,0,0],-1)
        cv2.circle(crop_img,topmost,5,[255,0,0],-1)
        cv2.circle(crop_img,leftmost,5,[255,0,0],-1)
        #Finding the required distance for angle 
        a1 = (end[0]-start[0])**2 + (end[1]-start[1])**2 
        b1 = (far[0]-start[0])**2 + (far[1]-start[1])**2 
        c1 = (end[0]-far[0])**2 + (end[1]-far[1])**2 
        #Finding Required distance for angle2
        a2 = math.sqrt((bottommost[0]-rightmost[0])**2 + (bottommost[1]-rightmost[1])**2 )
        b2 = math.sqrt((rightmost[0]-topmost[0])**2 + (rightmost[1]-topmost[1])**2 )
        c2 = math.sqrt((topmost[0]-bottommost[0])**2 + (topmost[1]-bottommost[1])**2 )
        #Finding Required Distance for angle3
        a3 = math.sqrt((rightmost[0]-leftmost[0])**2 + (rightmost[1]-leftmost[1])**2 )
        b3 = math.sqrt((leftmost[0]-topmost[0])**2 + (leftmost[1]-topmost[1])**2 )
        c3 = math.sqrt((topmost[0]-rightmost[0])**2 + (topmost[1]-rightmost[1])**2 )
        #Finding angle2 and angle3
        an1 = (b2**2 + a2**2 - c2**2)/(2*b2*a2)
        angle2 = (math.acos(an1) * 57)

        an2 = (b3**2 + a3**2 - c3**2)/(2*b3*a3)
        angle3 = (math.acos(an2) * 57)


        #Determing the fingers which is raised using angle2 and angle3

        # for 0,1,6 and 9
        if angle2 >=96 and angle2<=105:
            cnt2 = 0
        elif angle2 >=125 and angle2 <=140:
            cnt2 = 1
        elif angle2 >=80 and angle2<=95:
            cnt2 = 6
        elif angle3>=60 :
            cnt2 = 9
        # for 2 and 7
        if angle3>=50 :
            cnt3 = 7
        else:
            cnt3 = 2
        #for 3 and 8
        if angle3 >=40 and angle3 <= 70:
            cnt4 = 8
        else :
            cnt4 = 3
        # for 4 
        if angle2>=80 and angle2<=100:
            cnt5 = 4
        #Finding the fingers using convexity defects 
        if(a1>=0 and b1>=0 and c1>=0) :
            a = math.sqrt(a1)
            b = math.sqrt(b1)
            c = math.sqrt(c1)
            an = (b**2 + c**2 - a**2)/(2*b*c)
            if an >= 0.0 and an <= 1.0:
                angle = (math.acos(an) * 57)
                if angle <= 90  :
                    cnt += 1
    #counter to find fingers raised and then seperating them using angles
    if cnt>=0:
        cnt = cnt + 1
    #counting the fingers
    if cnt == 1 :
        cv2.putText(crop_img,str(cnt2),(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
        cnt6 = cnt2
    elif cnt == 2:
        cv2.putText(crop_img,str(cnt3),(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
        cnt6 = cnt3
    elif cnt == 3:
        cv2.putText(crop_img,str(cnt4),(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
        cnt6 = cnt4
    elif cnt == 4:
        cv2.putText(crop_img,str(cnt5),(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
        cnt6 = cnt5
    elif cnt == 5:
        cv2.putText(crop_img,'5',(0,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
        cnt6 = 5
    #To move forward
    while cnt6 == 1:
        rospy.init_node('gazebo', anonymous=True)
        cmd_vel_topic='/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        velocity_message = Twist()
        velocity_message.linear.x=0.1
        velocity_message.linear.y=0
        velocity_message.linear.z=0
        velocity_message.angular.x=0
        velocity_message.angular.y=0
        velocity_message.angular.z=0
        velocity_publisher.publish(velocity_message)
        time.sleep(0.033)
        break
    #To move backward
    while cnt6 == 2:
        rospy.init_node('gazebo', anonymous=True)
        cmd_vel_topic='/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        velocity_message = Twist()
        velocity_message.linear.x=-0.1
        velocity_message.linear.y=0
        velocity_message.linear.z=0
        velocity_message.angular.x=0
        velocity_message.angular.y=0
        velocity_message.angular.z=0
        velocity_publisher.publish(velocity_message)
        time.sleep(0.033)
        break
    #To stop
    while cnt6 == 4:
        rospy.init_node('gazebo', anonymous=True)
        cmd_vel_topic='/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        velocity_message = Twist()
        velocity_message.linear.x=0
        velocity_message.linear.y=0
        velocity_message.linear.z=0
        velocity_message.angular.x=0
        velocity_message.angular.y=0
        velocity_message.angular.z=0
        velocity_publisher.publish(velocity_message)
        time.sleep(0.033)
        break
    #Rotate in anticlockwise
    while cnt6 == 5:
        rospy.init_node('gazebo', anonymous=True)
        cmd_vel_topic='/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        velocity_message = Twist()
        velocity_message.linear.x=0
        velocity_message.linear.y=0
        velocity_message.linear.z=0
        velocity_message.angular.x=0
        velocity_message.angular.y=0
        velocity_message.angular.z=0.5
        velocity_publisher.publish(velocity_message)
        time.sleep(0.033)
        break
    #Rotate in clockwise
    while cnt6 == 3:
        rospy.init_node('gazebo', anonymous=True)
        cmd_vel_topic='/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        velocity_message = Twist()
        velocity_message.linear.x=0
        velocity_message.linear.y=0
        velocity_message.linear.z=0
        velocity_message.angular.x=0
        velocity_message.angular.y=0
        velocity_message.angular.z=-0.5
        velocity_publisher.publish(velocity_message)
        time.sleep(0.033)
        break
    
    cv2.imshow('final',crop_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
