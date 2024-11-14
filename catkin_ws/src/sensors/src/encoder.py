#!/usr/bin/env python3
import struct
import rospy
import roslib
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_msgs.msg import Int16
import time
import math
import RPi.GPIO as gpio


global YAWRATE, HEADING, DIR
YAWRATE = 0
HEADING = 0
DIR = 0

def yawcallback(data):
    global YAWRATE
    YAWRATE = data.data

def headingcallback(data):
    global HEADING
    HEADING = data.data

def dircallback(data):
    global DIR
    DIR = data.data

def main():
    # Create Publisher Instance
    x_pub = rospy.Publisher('/sensors/x', Float64, queue_size=1)
    y_pub = rospy.Publisher('/sensors/y', Float64, queue_size=1)
    inHome_pub = rospy.Publisher('/robot_control/inHome', Bool, queue_size = 10)
    inConstruction_pub = rospy.Publisher('robot_control/inConstruction', Bool, queue_size = 10)
    rospy.init_node('Encoder_Node', anonymous=False)

    # Create Subscribers
    #rospy.Subscriber('sensors/yaw_rate', Float64, yawcallback)
    rospy.Subscriber('sensors/heading', Float64, headingcallback)
    rospy.Subscriber('path_planner/dir', Int16, dircallback)

    # Init Hardware
    gpio.setmode(gpio.BCM)
    gpio.setup(20, gpio.IN)
    gpio.setup(21, gpio.IN)
    gpio.setup(12, gpio.IN)
    gpio.setup(1, gpio.IN)

    # Init Variables
    encoderA = gpio.input(20)
    encoderB = gpio.input(21)
    encoderC = gpio.input(12)
    encoderD = gpio.input(1)
    x_pos = 0.5 * 304.8 #start x position in absolute coordinate frame (mm)
    y_pos = 2 * 304.8 #stary y position in absolute coordinate frame (mm)
    positionAB = 0
    positionCD = 0
    counterAB_prev = 0
    counterCD_prev = 0
    counterAB = 0
    counterCD = 0
    last_AB = 0b00
    last_CD = 0b00
    travelled = 0
    last_counter_AB, last_counter_CD = 0, 0
    outcome = [0, -1, 1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0]

    # Init Accelerometer Variables
    heading = 0
    z_rot = 0
    
    # Init extra
    rate = rospy.Rate(1000)
    inHome = 0
    inConstruction = 0

    while not rospy.is_shutdown():
        
        # Collect data from Hardware
        encoderA = gpio.input(20)
        encoderB = gpio.input(21)
        encoderC = gpio.input(12)
        encoderD = gpio.input(1)

        # Read Encoder Movement and update global position
        current_AB = (encoderA << 1) | encoderB
        current_CD = (encoderC << 1) | encoderD
        positionAB = (last_AB << 2) | current_AB
        positionCD = (last_CD << 2) | current_CD
        counterAB += outcome[positionAB]
        counterCD += outcome[positionCD]
        last_AB = current_AB
        last_CD = current_CD
        dAB = counterAB - last_counter_AB
        dCD = counterCD - last_counter_CD
            
        # If the robot is not rotating, update position
        if (DIR == 0):
            travelled = ((counterAB - counterAB_prev) + (counterCD - counterCD_prev))/2 * 6.28/150 * 45
            dx = travelled * math.cos(HEADING)
            dy = travelled * math.sin(HEADING)
            x_pos = x_pos - dx
            y_pos = y_pos + dy
            counterAB_prev = counterAB
            counterCD_prev = counterCD

        # Figure out if in home or in construction
        if x_pos < (2 * 304.8):
            if y_pos > (2 * 304.8):
                inHome = 1
                inConstruction = 0
                print('In Home')
            else:
                inHome = 0
                inConstruction = 1
                print('In Construction')
        else:
            inHome = 0
            inConstruction = 0

        # Publish Data
        x_pub.publish(x_pos)
        y_pub.publish(y_pos)
        inHome_pub.publish(inHome)
        inConstruction_pub.publish(inConstruction)


        # Print Statement for debugging
        #print("X Position (mm): " + str(x_pos) + "     Y Position (mm): " + str(y_pos))
        
        # Prepare for next iteration
        last_counter_AB = counterAB
        last_counter_CD = counterCD
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
