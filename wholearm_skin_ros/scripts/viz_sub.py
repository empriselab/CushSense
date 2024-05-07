#! /usr/bin/env python 

import rospy
# Other imports
import rospy
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import random
import tf
import tf2_ros
import numpy as np
import matplotlib.pyplot as plt
import time
import rospy
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import json
# Program Code

time.sleep(10)
cmap = plt.cm.Reds

pub = rospy.Publisher("taxel_markers", MarkerArray, queue_size=10)
#x,y,z,roll,pitch,yaw

link_names = ['shoulder_link', 'half_arm_1_link', 'half_arm_2_link', 'forearm_link', 'spherical_wrist_1_link', 'spherical_wrist_2_link']
robot = dict()
#import robot.json
with open('/home/robot.json') as json_file:
    robot = json.load(json_file)

# convert pos to geometry_msgs/Pose
for e in range(len(link_names)):
    for i, pose in enumerate(robot[link_names[e]]):
        new_pos = PoseStamped()
        new_pos.header.frame_id = e
        new_pos.pose.position.x = pose[0]
        new_pos.pose.position.y = pose[1]
        new_pos.pose.position.z = pose[2] 
        new_pos.pose.orientation.x = pose[3]
        new_pos.pose.orientation.y = pose[4]
        new_pos.pose.orientation.z = pose[5]
        new_pos.pose.orientation.w = pose[6]
        robot[link_names[e]][i] = new_pos
   
rospy.init_node('calibration_subscriber')

tf2_buffer = tf2_ros.Buffer()
tf2_listener = tf2_ros.TransformListener(tf2_buffer)

def callback(data):
    viz_pose = [] 
    for e in reversed(range(len(link_names))):
        try:
            tran = tf2_buffer.lookup_transform("base_link", link_names[e], rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        for i in (range(len(robot[link_names[e]]))):  
            transformed_pos = do_transform_pose(robot[link_names[e]][i], tran)
            quaternion = [transformed_pos.pose.orientation.x, transformed_pos.pose.orientation.y, transformed_pos.pose.orientation.z, transformed_pos.pose.orientation.w]
            q_new = quaternion
            viz_pose.append([transformed_pos.pose.position.x, transformed_pos.pose.position.y, transformed_pos.pose.position.z, q_new[0], q_new[1], q_new[2], q_new[3]])
           
    marker_array = MarkerArray()
    for i, value in enumerate(data.data):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        min = 0
        max = 100
        normalized = np.abs((value - min) / (max - min))
        if normalized >=.7:
            normalized = .7
        elif normalized < .05:
            normalized = .005
        color = cmap(normalized*3)
        marker.color.a = 1
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2] 
        marker.scale.x = normalized
        marker.scale.y = .0075
        marker.scale.z = .0075
        marker.pose.position.x = viz_pose[i][0]
        marker.pose.position.y = viz_pose[i][1]
        marker.pose.position.z = viz_pose[i][2]
        # rotate orientation by 90 degrees around z axis
        quaternion = [viz_pose[i][3], viz_pose[i][4], viz_pose[i][5], viz_pose[i][6]]
        q_new = quaternion
        marker.pose.orientation.x = q_new[0]
        marker.pose.orientation.y = q_new[1]
        marker.pose.orientation.z = q_new[2]
        marker.pose.orientation.w = q_new[3]
        marker.id = i
        marker_array.markers.append(marker)
    pub.publish(marker_array)

rospy.Subscriber("/calibration", Float32MultiArray, callback)
rospy.spin()
# End of Code