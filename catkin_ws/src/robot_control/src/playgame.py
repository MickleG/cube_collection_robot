#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
from gpiozero import Button
from gpiozero import Servo
from gpiozero import RGBLED
import time

global COLOR, INHOME, INCONSTRUCTION
COLOR = None

# ROS Subscriber Callbacks
def colorcallback(data):
    global COLOR
    COLOR = data.data

def inHomecallback(data):
    global INHOME
    INHOME = data.data

def inConstructioncallback(data):
    global INCONSTRUCTION
    INCONSTRUCTION = data.data


def sortblock(color, color_prev, construction_color, servo):
    # Turn the sorter based on color of block
    if not (color == color_prev):
        if (color == construction_color):
            servo.max()
        elif not(color == 4):
            servo.min()
        else:
            pass

        time.sleep(1)
        servo.mid()


def main():
    # Initialize ROS Stuff
    rospy.init_node('Central_Node', anonymous=False)
    rospy.Subscriber('color_detected', UInt8, colorcallback)
    rospy.Subscriber('/robot_control/inHome', Bool, inHomecallback)
    rospy.Subscriber('/robot_control/inConstruction', Bool, inConstructioncallback)
    drive_home_pub = rospy.Publisher('robot_control/drive_home', Bool, queue_size = 10)
    drive_construction_pub = rospy.Publisher('robot_control/drive_construction', Bool, queue_size = 10)
    run_intake_pub = rospy.Publisher('robot_control/run_intake', Bool, queue_size = 10)
    rate = rospy.Rate(10)

    # Initialize System Inputs
    timeEnd = 0
    timeLow = 0
    inHome = 0
    inConstruction = 0
    state = 0
    start_game = False
    cnt = 0
    run_intake = 0
    drive_home = 0
    drive_construction = 0

    # Initialize Hardware
    button1 = Button(14, pull_up=True)
    button2 = Button(15, pull_up=True)
    button3 = Button(24, pull_up=True)
    desired_led = RGBLED(red = 26, green = 19, blue = 13, active_high = 1)
    current_led = RGBLED(red = 6, green = 5, blue = 0, active_high = 1)
    servo = Servo(7, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
    

    # Initialize Other Variables
    construction_color = None
    color_prev = None
    while not rospy.is_shutdown():
        rate.sleep()
        #check for button inputs
        b1 = button1.is_pressed
        b2 = button2.is_pressed
        b3 = button3.is_pressed
        print(str(b1) + str(b2) + str(b3))

        color = COLOR
        print("Color = " + str(COLOR))
        
        # Determine Time in the game
        if button3.is_pressed:
            cnt = cnt + 0.1
        
        # Take color input and set the LED color if button3 is pressed
        des_color = (b1 << 1) + b2
        if button3.is_pressed and not start_game and cnt < 120:
            construction_color = des_color
            if construction_color == 0:
                desired_led.color = (1, 0, 0)
            elif construction_color == 1:
                desired_led.color = (1, 1, 0)
            elif construction_color == 2:
                desired_led.color = (0, 1, 0)
            elif construction_color == 3:
                desired_led.color = (0, 0, 1)
            start_game = True
        else:
            sortblock(color, color_prev, construction_color, servo)
        
        # Choose color of second LED based on location on playing field
        if INHOME == 1:
            current_led.color = (1, 0, 0)
        elif INCONSTRUCTION == 1:
            current_led.color = (0, 1, 0)
        else:
            current_led.color = (0, 0, 1)

        color_prev = color # Save current LED color

        # If out of time, turn the desired color LED off
        if timeEnd:
            desired_led.color = (0, 0, 0)
            timeEnd = 0
            drive_home = 0
            run_intake = 0

        # Get FSM Inputs from Subscribers
        if cnt > 100:
            timeLow = 1
        elif cnt >= 120:
            timeEnd = 1
            timeLow = 0

        if button3.is_pressed:
            run_intake = 1
        else:
            run_intake = 0

        inHome = INHOME
        inConstruction = INCONSTRUCTION

        ###
        fsm_inputs = (timeEnd << 4) + (timeLow << 3) + (inHome << 2) + (inConstruction << 1) + run_intake
        #print("FSM Input Array: " + str(fsm_inputs))
        if True:
            if(state == 0):
                # Game Start
                output = 0b000
                if (timeEnd):
                    state = 0
                elif run_intake:
                    state = 1

            if(state == 1):
                # Follow Path and run intake
                output = 0b001
                if (timeEnd):
                    state = 0
                elif (timeLow):
                    state = 2

            if(state == 2):
                # Go Home
                output = 0b100
                if (timeEnd):
                    state = 0

            # Removed go to construction state for demo

            # Gather boolean values for each output needed
            drive_home = (output & 0b100) >> 2
            drive_construction = (output & 0b010) >> 1
            run_intake = (output & 0b001)

            # Publish outputs
            run_intake_pub.publish(run_intake)
            drive_home_pub.publish(drive_home)
            drive_construction_pub.publish(drive_construction)

    # Sleep until next iteration
    rate.sleep()


if __name__ == '__main__':
    main()

