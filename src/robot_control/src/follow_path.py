#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from gpiozero import PWMOutputDevice
from gpiozero import Motor
import time
import numpy as np
import math

#init variables
global DRIVE_HOME, DRIVE_CONSTRUCTION, X_COORD, Y_COORD, HEADING, RUNINTAKE
DRIVE_HOME = 0
DRIVE_CONSTRUCTION = 0
RUNINTAKE = 0
X_COORD = 0.5 * 304.8
Y_COORD = 2 * 304.8
HEADING = 0
START_GAME = 1


# ROS Subscriber Callbacks
def drivehomecallback(data):
    global DRIVE_HOME
    DRIVE_HOME = data.data

def driveconstructioncallback(data):
    global DRIVE_CONSTRUCTION
    DRIVE_CONSTRUCTION = data.data

def runintakecallback(data):
    global RUNINTAKE
    RUNINTAKE = data.data

def xcoordcallback(data):
    global X_COORD
    X_COORD = data.data

def ycoordcallback(data):
    global Y_COORD
    Y_COORD = data.data

def headingcallback(data):
    global HEADING
    HEADING = data.data

# Hard-Coded Triangle wave path
def brainlesspath(gamestart):
    global START_GAME

    # Init variables 
    desired_y_travel = 1 #ft -- set to 0.5 for path that goes to construction
    desired_x_travel = 1 #ft
    field_width = 0.5 #ft
    track_width = 0.5 #ft
    start_x = 7 * 304.8 #mm
    desired_y_top = (4 - desired_y_travel) / 2 * 304.8 #mm
    goaly = desired_y_top
    desired_y_bottom = (4 * 304.8 - desired_y_top) #mm
    theta = math.atan(desired_y_travel / desired_x_travel)
    
    # Start by moving to the edge of the arena and then make a right turn, going up 
    if gamestart == 0:
        if abs(X_COORD - start_x) < 17.7:
            goalx = start_x
            goaly = desired_y_top
            goalh = 90 * 3.14 / 180
        else:
            goalx = start_x
            goaly = 0
            goalh = 0
    # Follow the path, watching X location of robot to decide when to begin next leg of the path
    elif X_COORD > start_x - (desired_x_travel) * 304.8:
        START_GAME = 0
        goalx = start_x - (desired_x_travel) * 304.8
        goaly = desired_y_bottom
        goalh = 3.14 + theta
    elif X_COORD > start_x - (2 * desired_x_travel) * 304.8:
        goalx = start_x - (2 * desired_x_travel) * 304.8
        goaly = desired_y_top
        goalh = 3.14 - theta
    elif X_COORD > start_x - (3 * desired_x_travel) * 304.8:
        goalx = start_x - (3 * desired_x_travel) * 304.8
        goaly = desired_y_bottom
        goalh = 3.14 + theta
    elif X_COORD > start_x - (4 * desired_x_travel) * 304.8:
        goalx = start_x - (4 * desired_x_travel) * 304.8
        goaly = desired_y_top
        goalh = 3.14 - theta
    elif X_COORD > start_x - (5 * desired_x_travel) * 304.8:
        goalx = start_x - (5 * desired_x_travel) * 304.8
        goaly = desired_y_bottom
        goalh = 3.14 + theta
    elif X_COORD > 4.0 * 304.8:
        goalx = 4.0 * 304.8
        goaly = desired_y_top
        goalh = 3.14 - theta
    elif X_COORD > 3.5 * 304.8:
        goalx = 3.5 * 304.8
        goaly = desired_y_bottom
        goalh = 3.14 + theta
    elif X_COORD > 3.0 * 304.8:
        goalx = 3.0 * 304.8
        goaly = desired_y_top
        goalh = 3.14 - theta
    elif X_COORD > 2.5 * 304.8:
        goalx = 2.5 * 304.8
        goaly = desired_y_bottom
        goalh = 3.14 + theta
    elif X_COORD > 2.0 * 304.8:
        goalx = 2.0 * 304.8
        goaly = desired_y_top
        goalh = 3.14 - theta
    elif X_COORD > 1.5 * 304.8:
        goalx = 1.5 * 304.8
        goaly = desired_y_bottom
        goalh = 3.14 + theta
    else:
        goalx = 0
        goaly = -1000
        goalh = 0

    # Print values for debugging
    print("Goal X: " + str(goalx / 304.8))
    print("Goal Y: " + str(goaly / 304.8))
    print("Goal H: " + str(goalh * 180/3.14))
    return goalx, goaly, goalh

def drivehome():
    # Direct the robot to the center of the home square
    goalx = 304.8
    goaly = 304.8*3
    goalh = -3.14/2 - math.atan2((X_COORD - goalx), (Y_COORD, goaly))
    print(goalh)
    return goalx, goaly, goalh

def driveconstruction():
    # Direct the robot to the center of the construction square
    goalx = 304.8
    goaly = 304.8
    goalh = math.atan2((Y_COORD - goaly), (X_COORD - goalx))

    return goalx, goaly, goalh

def main():
    # Initialize ROS Stuff
    rospy.init_node('Follow_Path_Node', anonymous=False)
    rospy.Subscriber('robot_control/drive_home', Bool, drivehomecallback)
    rospy.Subscriber('robot_control/drive_construction', Bool, driveconstructioncallback)
    rospy.Subscriber('robot_control/run_intake', Bool, runintakecallback)
    rospy.Subscriber('sensors/x', Float64, xcoordcallback)
    rospy.Subscriber('sensors/y', Float64, ycoordcallback)
    rospy.Subscriber('sensors/heading', Float64, headingcallback)
    dirpub = rospy.Publisher('path_planner/dir', Int16, queue_size = 1)
    rate = rospy.Rate(10) # Hz

    # Init Hardware
    lmotor = Motor(17, 18)
    rmotor = Motor(22, 23)

    # Init Other Variables
    gamestart = 0
    turn = 0
    goup = 0
    drive = 0
    prevx = 0
    prevy = 0
    prevh = 0
    guh = 0
    goalx = 0
    goaly = 0
    goalh = 0

    while not rospy.is_shutdown():
        

        # Figure out if we've driven to the end of the line yet
        if (Y_COORD < 1.5 * 304.8):
            gamestart = 1


        # Decide what path to follow based on FSM
        if RUNINTAKE == 1:
            goalx, goaly, goalh = brainlesspath(gamestart)
        elif DRIVE_HOME == 1 and START_GAME == 0:
            goalx, goaly, goalh = drivehome()
        elif DRIVE_CONSTRUCTION == 1 and START_GAME == 0:
            goalx, goaly, goalh = driveconstruction()
        else:
            # Go nowhere
            goaly = -1000
            print("Unknown Path")

        
        # Calculate error to control rotation speed while turning
        error = (HEADING - goalh)
        if abs(error) > .3:
            mspeed = .2
        else:
            mspeed = abs(HEADING-goalh)
        
        # Actuate Motors
        # Rotate before moving to avoid confusion in location
        if goaly == -1000:
            pass
        elif (HEADING - goalh) > 0.1: # Turn Left
            lmotor.backward(mspeed)
            rmotor.backward(mspeed)
            guh = -1
            print("Turn Left")
        elif (HEADING - goalh) < -0.1: # Turn Right
            lmotor.forward(mspeed)
            rmotor.forward(mspeed)
            guh = 1
            print("Turn Right")
        elif (HEADING - goalh) > 0.05: # Turn Left Small
            lmotor.backward(mspeed + 0.1)
            rmotor.forward(mspeed)
            guh = 0
            print("Turn Left a little")
        elif (HEADING - goalh) < -0.05:
            lmotor.backward(mspeed)
            rmotor.forward(mspeed + 0.1)
            guh = 0
            print("Turn Right a little")
        else:
            lmotor.backward(0.5)
            rmotor.forward(0.5)
            guh = 0
            print("Forward but stupid")

        print("X: " + str(X_COORD/304.8) + "    Y: " + str(Y_COORD/304.8) + "    Heading: " + str(HEADING * 180/3.14))
        
        # Publish ROS messages
        dirpub.publish(guh)

        # Prepare for next iteration
        rate.sleep()
    

if __name__ == '__main__':
    main()

