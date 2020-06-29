#!/usr/bin/env python
#author: Vu Ngoc Son
#gmail: sonvungoc96@gmail.com


import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import time
from std_msgs.msg import String


x_min = 0.0
y_min = 0.0
x_max = 11.0
y_max = 11.0
PI = 3.14159265359

velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
turtlesim_pose = Pose()


def move( speed, distance, isForward):
    #declare a Twist message to send velocity commands
    vel_msg = Twist()


    ##set a random linear velocity in the x-axis
    if isForward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)

    vel_msg.linear.y = 0
    vel_msg.linear.z = 0

    ##set a random angular velocity in the y-axis
    vel_msg.angular.x =0
    vel_msg.angular.y =0
    vel_msg.angular.z =0

    t0 = rospy.get_time()
    current_distance =0
    loop_rate = rospy.Rate(100) 

    while True:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.get_time()
        current_distance = speed*(t1-t0)
        loop_rate.sleep()

        if not (current_distance<distance):
            rospy.loginfo("reached")
            break

    vel_msg.linear.x = 0.0
    velocity_publisher.publish(vel_msg)



def rotate(angular_speed, relative_angle, clockwise):
    #declare a Twist message to send velocity commands
    vel_msg = Twist()

    #set a random linear velocity in the x-axis
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    #set a random linear velocity in the y-axis
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z =  abs(angular_speed)

    current_angle = 0.0
    t0 = rospy.get_time()
    loop_rate = rospy.Rate(10)
    while True:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.get_time()
        current_angle = angular_speed * (t1 - t0)
        loop_rate.sleep()
        if not (current_angle<relative_angle):
            break

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def degrees2radians(angle_in_degrees):
    return (angle_in_degrees*PI/180.0)


def update_pose(pose_message):
        turtlesim_pose.x = pose_message.x
        turtlesim_pose.y = pose_message.y
        turtlesim_pose.theta = pose_message.theta


def getDistance(x1, y1, x2, y2):
    return sqrt(pow((x1-x2),2)+pow((y1-y2),2))


def  setDesiredOrientation (desired_angle_radians):
    clockwise = False
    relative_angle_radians = desired_angle_radians - turtlesim_pose.theta
    if (relative_angle_radians < 0):
        clockwise = True
    else:
        clockwise = False

    rotate (degrees2radians(10), abs(relative_angle_radians), clockwise)


def moveGoal(goal_pose, distance_tolerance):
    #declare a Twist message to send velocity commands
    vel_msg = Twist()
    loop_rate = rospy.Rate(100)
    E= 0.0

    while True:
         ################### Proportional Controller ###################
        ##linear velocity in the x-axis
        Kp=1.0
        Ki=0.02
        #double v0 = 2.0;
        alpha =4
        e = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)
        E = E+e
        
        #Kp = v0 * (exp(-alpha)*error*error)/(error*error);
        vel_msg.linear.x = (Kp*e)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        #angular velocity in the z-axis
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = alpha*(atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x)-turtlesim_pose.theta)

        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()

        distance_moved =  getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x,goal_pose.y)

        if not (distance_moved>distance_tolerance):
                print ("end move goal")
                break

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


def spiralClean():
    vel_msg = Twist()
    loop_rate =rospy.Rate(1)
    constant_speed = 4
    vk = 1
    wk = 2
    rk = 0.5

    while((turtlesim_pose.x<10.5 and turtlesim_pose.y<10.5)):
        rk = rk + 1.0
        vel_msg.linear.x = rk
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        #set a random angular velocity in the y-axis
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z =constant_speed #((vk)/(0.5+rk))

        print ("vel_msg.linear.x =", vel_msg.linear.x)
        print ("vel_msg.angular.z =", vel_msg.linear.x)
        loop_rate.sleep()
        print (rk,", ",vk,", ",wk)

    vel_msg.linear.x =0
    velocity_publisher.publish(vel_msg)

def gridClean():
    pose = Pose()
    loop_rate = rospy.Rate(0.5)

    pose.x = 1
    pose.y = 1
    pose.theta =0

    moveGoal(pose, 0.01)
    loop_rate.sleep()
    setDesiredOrientation(0)
    loop_rate.sleep()


    move(2.0, 9.0, True)
    loop_rate.sleep()
    rotate(degrees2radians(10), degrees2radians(90), False)
    loop_rate.sleep()
    move(2.0, 9.0, True)

    rotate(degrees2radians(10), degrees2radians(90), False)
    loop_rate.sleep()
    move(2.0, 1.0, True)
    rotate(degrees2radians(10), degrees2radians(90), False)
    loop_rate.sleep()
    move(2.0, 9.0, True)


    rotate(degrees2radians(30), degrees2radians(90), True)
    loop_rate.sleep()
    move(2.0, 1.0, True)
    rotate(degrees2radians(30), degrees2radians(90), True)
    loop_rate.sleep()
    move(2.0, 9.0, True)

    distance = getDistance(turtlesim_pose.x, turtlesim_pose.y, x_max, y_max)




if __name__ == '__main__':
    try:
        rospy.init_node('robot_cleaner', anonymous=True)
        pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, update_pose)
        loop_rate = rospy.Rate(0.5)
        gridClean()
        spiralClean()
        loop_rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")