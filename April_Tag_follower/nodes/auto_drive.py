#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from math import atan2, sqrt
import numpy as np

class TagFollower:
    def __init__(self):
        rospy.init_node('turtlebot3_drive')
        self.rate = rospy.Rate(10) 
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.tag_detected = False
        self.tag_id=None
        self.tag_x = 0.0
        self.tag_y = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

    def tag_callback(self, data):
        if data.detections:
            self.tag_detected = True
            # Assume only one tag is being detected
            self.tag_id = data.detections[0].id
            self.tag_x = data.detections[0].pose.pose.pose.position.x
            self.tag_y = data.detections[0].pose.pose.pose.position.y
            rospy.loginfo("Tag ID: %d" % self.tag_id)
            rospy.loginfo("Tag positon: " + str(self.tag_x)+"  " +str(self.tag_y))
        else:
            self.tag_detected = False
            rospy.loginfo("No tag detected")

    def odom_callback(self, data):
        # Update the robot's current pose
        self.robot_x = data.pose.pose.position.x
        self.robot_y = data.pose.pose.position.y
        orientation = data.pose.pose.orientation
        (roll, pitch, self.robot_yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    def follow_tag(self):
     	
        while not rospy.is_shutdown():
            if self.tag_detected:
                # Calculate relative position of tag
                tag_rel_x =  self.tag_x
                tag_rel_y =  self.tag_y

                # Calculate distance and angle to tag
                distance_to_tag = sqrt(tag_rel_x**2 + tag_rel_y**2)
                angle_to_tag = atan2(tag_rel_y, tag_rel_x)
                angle_to_tag=np.interp(abs(angle_to_tag),(0,3.14),(-180,180))
                angular_speed=np.interp(angle_to_tag,(-180,180),(-2.84,2.84))
                # print("Anglular speed: "+str(angular_speed))
                # print("Distance:"+str(distance_to_tag))
                
                # Adjust linear and angular velocities based on distance and angle to tag
                linear_speed = np.clip(distance_to_tag,0,0.22) #if distance_to_tag > 0.3 else 0.0  # Stop moving if close to the tag
                angular_speed=angular_speed*0.2
                print("Going to tag with speed: ",linear_speed,"  ",angular_speed)

                # Publish control commands
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = linear_speed
                cmd_vel_msg.angular.z = angular_speed
                self.cmd_vel_pub.publish(cmd_vel_msg)

            if not self.tag_detected:
                # Stop the robot if no tag is detected
                cmd_vel_msg = Twist()
                self.cmd_vel_pub.publish(cmd_vel_msg)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        follower = TagFollower()
        follower.follow_tag()

    except rospy.ROSInterruptException:
        pass
