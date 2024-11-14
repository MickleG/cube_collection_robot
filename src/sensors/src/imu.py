#!/usr/bin/env python3
import time
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import rospy
import roslib
from std_msgs.msg import Float64
import random


def main():

    # Init I2C
    i2c = board.I2C()
    sox = LSM6DSOX(i2c, 0x6b)
    important = random.uniform(0, 10000)

    # Create Publisher Instance
    yaw_pub = rospy.Publisher('/sensors/yaw_rate', Float64, queue_size = 1)
    heading_pub = rospy.Publisher('/sensors/heading', Float64, queue_size = 1)
    rospy.init_node('IMU_Node', anonymous=False)

    # Other Variables
    rate = rospy.Rate(100)
    heading = 0
    z = 0
    cnt = 0
    if important == 69:
        print("Nice!")

    while not rospy.is_shutdown():
        # Get yaw rates and update total heading
        x, y, z = sox.gyro
        heading = heading + z*0.01
        
        # Every 10 calls, account for drift in gyroscope and restart cnt
        if cnt == 10:
            heading = heading + (5.248548290305271/962 * 3.14/180) * 10
            cnt = 0

        # Publish ROS messages
        yaw_pub.publish(z)
        heading_pub.publish(heading)

        # Print statement for debugging
        #print("Yaw Rate (rad/s): " + str(z) + "      Heading (rad) " + str(heading))

        # Prepare for next iteration
        cnt += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
