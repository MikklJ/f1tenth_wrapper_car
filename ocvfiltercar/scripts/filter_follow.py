#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from os import path
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from move_robot import MoveRosBots
from geometry_msgs.msg import Twist

def callback(x):
    pass

class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.datapath = "/home/michaelji/tritonai/catkin_ws/src/ocvfiltercar/data/records_1/"

        #self.image_sub = rospy.Subscriber("/robot1/camera1/image_raw",Image,self.camera_callback)
        self.image_sub = rospy.Subscriber("/image", Image, self.camera_callback)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

        self.moverosbots_object = MoveRosBots()

    def camera_callback(self,data):

        """
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        """
        cv2.namedWindow('sliders')

        # HSV filter for isolating all lines
        bestfilter = {
            "lowH": 16,
            "highH": 43,
            "lowS": 51,
            "highS": 140,
            "lowV": 42,
            "highV": 213
        }
        # RGB for mask to isolate yellow center line
        rgbcenterfilter = {
            "lowR": 200,
            "highR": 255,
            "lowG": 100,
            "highG": 255,
            "lowB": 100,
            "highB": 170
        }
        # RGB for mask to isolate white borders
        rgbsidefilter = {
            "lowR": 200,
            "highR": 255,
            "lowG": 200,
            "highG": 255,
            "lowB": 200,
            "highB": 255
        }

        # Load filter values
        lowR = rgbcenterfilter.get("lowR")
        highR = rgbcenterfilter.get("highR")
        lowG = rgbcenterfilter.get("lowG")
        highG = rgbcenterfilter.get("highG")
        lowB = rgbcenterfilter.get("lowB")
        highB = rgbcenterfilter.get("highB")

        cv2.createTrackbar('lowR', 'sliders', lowR, 255, callback)
        cv2.createTrackbar('highR', 'sliders', highR, 255, callback)

        cv2.createTrackbar('lowG', 'sliders', lowG, 255, callback)
        cv2.createTrackbar('highG', 'sliders', highG, 255, callback)

        cv2.createTrackbar('lowB', 'sliders', lowB, 255, callback)
        cv2.createTrackbar('highB', 'sliders', highB, 255, callback)

        i = 0        
        #while path.exists(self.datapath + "img_" + str(i) + ".jpg"):
        if True:
            #cv_image = cv2.imread(self.datapath + "img_" + str(i) + ".jpg")
            print(data)
            try:
            # We select bgr8 because its the OpneCV encoding by default
                cv_image = self.bridge_object.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
                exit(1)

            height, width, channels = cv_image.shape
            crop_img = cv_image[int(height/2):height, 0:width]

            # Set mask values
            lowR = cv2.getTrackbarPos('lowR', 'sliders')
            highR = cv2.getTrackbarPos('highR', 'sliders')
            lowG = cv2.getTrackbarPos('lowG', 'sliders')
            highG = cv2.getTrackbarPos('highG', 'sliders')
            lowB = cv2.getTrackbarPos('lowB', 'sliders')
            highB = cv2.getTrackbarPos('highB', 'sliders')

            rgb = cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB)


            lower = np.array([lowR, lowG, lowB])
            higher = np.array([highR, highG, highB])
            mask = cv2.inRange(rgb, lower, higher)

            # Display images
            cv2.imshow('cv_image', cv_image)
            cv2.imshow('crop_img', crop_img)
            cv2.imshow('mask', mask)
            # Clean monitor positions
            cv2.moveWindow("mask", 0,900);
            cv2.moveWindow("crop_img", 0,400);
            cv2.moveWindow("cv_image", 0,700);

            # Calculate c_x, c_y
            # Center Line:
            # Calculate centroid of the blob of binary image using ImageMoments
            m = cv2.moments(mask, False)
            try:
                cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            except ZeroDivisionError:
                cy, cx = height/2, width/2

            # Draw Circle on resultant cropped image
            result =cv2.bitwise_and(crop_img,crop_img, mask = mask)
            cv2.circle(result,(int(cx), int(cy)), 5,(0,0,255),-1)
            cv2.imshow('result', result)
            cv2.moveWindow('result', 400, 0)

            error_x = cx - width / 2;
            angular_z = -error_x / 100;
            rospy.loginfo("ANGULAR VALUE SENT===>"+str(angular_z))
            a = AckermannDriveStamped()
            a.drive.speed = 2
            a.drive.steering_angle = angular_z
            self.drive_pub.publish(a)

            #twist_object = Twist();
            #twist_object.linear.x = 0.2;
            #twist_object.angular.z = -error_x / 100;

            # SIDES
            # Use gradients to determine steering
            # Load filter values
            """
            lowR = rgbsidefilter.get("lowR")
            highR = rgbsidefilter.get("highR")
            lowG = rgbsidefilter.get("lowG")
            highG = rgbsidefilter.get("highG")
            lowB = rgbsidefilter.get("lowB")
            highB = rgbsidefilter.get("highB")

            border_lower = np.array([lowR, lowG, lowB])
            border_higher = np.array([highR, highG, highB])
            border_mask = cv2.inRange(rgb, border_lower, border_higher)
            grad_x = np.diff(border_mask, n = 1, axis = 0)
            all_indices = [[]]
            print(grad_x)
            for row in grad_x:
                indices = []
                for index in range(len(row)):
                    if row[index] != 0:
                        print(str(index) + " ", end='')
                        np.append(indices, index)

                if len(indices) == 2:
                    np.append(all_indices, indices)

            print(all_indices)
            print(np.mean(np.diff(all_indices, axis = 0), axis = 0))

            cv2.imshow('gradx', grad_x)
            cv2.moveWindow('gradx', 400, 600)
            """
            i += 1
            
            
            key = cv2.waitKey(50) & 0xFF
            if key == ord('q'):
                exit(0)
            

        return

        # We get image dimensions and crop the parts of the image we don't need
        # Bear in mind that because the first value of the image matrix is start and second value is down limit.
        # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
        # To make process faster.
        height, width, channels = cv_image.shape
        rows_to_watch = 100
        top_trunc = 1*height / 2 #get 3/4 of the height from the top section of the image
        bot_trunc = top_trunc + rows_to_watch #next set of rows to be used
        crop_img = cv_image[top_trunc:bot_trunc, 0:width]
        
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV).astype(np.float)
        
        # Define the Yellow Colour in HSV
        #RGB
        #[[[222,255,0]]]
        #BGR
        #[[[0,255,222]]]
        """
        To know which color to track in HSV, put in BGR. Use ColorZilla to get the color registered by the camera
        >>> yellow = np.uint8([[[B,G,R ]]])
        >>> hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)
        >>> print( hsv_yellow )
        [[[ 34 255 255]]
        """
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        
        
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)

        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)
        cv2.imshow("MASK", mask)
        cv2.imshow("RES", res)
        
        cv2.waitKey(1)
        
        
        error_x = cx - width / 2;
        angular_z = -error_x / 100;
        rospy.loginfo("ANGULAR VALUE SENT===>"+str(angular_z))
        twist_object = Twist();
        twist_object.linear.x = 0.2;
        twist_object.angular.z = -error_x / 100;
        # Make it start turning
        self.moverosbots_object.move_robot(twist_object)
        
    def clean_up(self):
        self.moverosbots_object.clean_class()
        cv2.destroyAllWindows()
        
        

def main():
    rospy.init_node('line_following_node', anonymous=True)
    
    line_follower_object = LineFollower()
    
    #line_follower_object.camera_callback(3)

    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()

    
    
if __name__ == '__main__':
    main()