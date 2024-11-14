#!/usr/bin/env python3
import rospy
import cv2
import math
from std_msgs.msg import UInt8

threshold = 20
pixel_window_apothem = 50

CUTOFF = 160

def getColor():
    # Open Camera using OpenCV
    cam = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    # Init Variables
    total_b = 0
    total_g = 0
    total_r = 0

    # Capture image
    result, image = cam.read()

    if result:
        # Get image dimensions
        dimensions = image.shape
        height = dimensions[0]
        width = dimensions[1]
        height_middle = int(height / 2)
        width_middle = int(width / 2)

        # Look at only a small number of central pixels to get color
        for i in range(height_middle - pixel_window_apothem, height_middle + pixel_window_apothem):
            for j in range(width_middle - pixel_window_apothem, width_middle + pixel_window_apothem):
                total_b += image[i][j][0]
                total_g += image[i][j][1]
                total_r += image[i][j][2]

        num_pixels = (2 * pixel_window_apothem) ** 2

        # Get integer RGB values
        blue_val = math.floor(total_b / num_pixels)
        green_val = math.floor(total_g / num_pixels)
        red_val = math.floor(total_r / num_pixels)

        # Decide the color and return integer to signify color
        if red_val > CUTOFF and green_val < CUTOFF:
            return(0)
        elif red_val > CUTOFF and green_val > CUTOFF:
            return(1)
        elif green_val > CUTOFF and red_val < CUTOFF:
            return(2)
        elif blue_val > CUTOFF:
            return(3)
        else:
            return(4)

def publisher():
    pub = rospy.Publisher('color_detected', UInt8, queue_size=1)
    rospy.init_node('Color_Detect_Node', anonymous=True)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        color = getColor() # Get color
        pub.publish(color) # Publish color with ROS
        rate.sleep()       # Sleep until next iteration

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
