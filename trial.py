#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Twist
from math import atan2
from math import sqrt
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

angle = 0.0
distance = 0.0
destination_flag = 0
prev_angle = 0.0


def callback_for_angle(msg):
    global angle, prev_angle
    prev_angle = angle
    angle = msg.data
    print("msg for angle to turn: " + str(msg.data))
    GPSAlgoV1()


def callback_for_distance(msg):
    global distance
    distance = msg.data
    print("msg for distance to move: " + str(msg.data))
    GPSAlgoV1()


def GPSAlgoV1():
    global destination_flag, angle, distance, prev_angle
    velocity = Twist()
    x_linear = 0.3
    z_angular = 0.0
    orientation_error = float(angle)
    prev_orientation_error = float(prev_angle)
    kp = 0.01
    kd = 0.5

    if (distance <= 0.5 or destination_flag == 1):
        print("Destination reached!!")
        destination_flag = 1
        z_angular = 0
        x_linear = 0

    if (destination_flag == 0):
        if (abs(orientation_error) >= 2):
            z_angular = kp * orientation_error + kd * (orientation_error - prev_orientation_error)
            x_linear = 0.2  # Take slow turn while moving forward
            if (abs(orientation_error) > 10):
                x_linear = 0  # Rotate at the present location as error is much larger
                print("Rotating...")
            if (distance <= 1):
                x_linear = 0  # Stop
                z_angular = 0
                destination_flag = 1
                print("Destination reached!!")

        elif (abs(orientation_error) < 2):
            z_angular = 0  # Don't rotate as error is much smaller'
            if (distance > 1):
                x_linear = 0.4  # Move straight to goal
                print("Moving straight to goal")
            elif (distance <= 1):
                x_linear = 0  # Stop
                z_angular = 0
                destination_flag = 1
                print("Destination reached!!")

        if (abs(z_angular) > 0.5):
            z_angular = 0.5 * (abs(z_angular) / z_angular)

    velocity.angular.z = z_angular
    velocity.linear.x = x_linear
    pub.publish(velocity)


if __name__ == '__main__':
    try:
        rospy.init_node("parc_solutions", anonymous=True)
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        sub1 = rospy.Subscriber("/angle", Float64, callback_for_angle)
        sub2 = rospy.Subscriber("/distance", Float64, callback_for_distance)

        # Initialize publishers for /distance and /angle topics
        distance_pub = rospy.Publisher("/distance", Float64, queue_size=10)
        angle_pub = rospy.Publisher("/angle", Float64, queue_size=10)

        rospy.spin()
    except rospy.ROSInterruptException:
        velocity = Twist()
        velocity.angular.z = 0
        velocity.linear.x = 0
        pub.publish(velocity)
