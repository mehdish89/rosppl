#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from vicpark_msgs.msg import TreesStamped, Tree
from vicpark_msgs.msg import LandmarksStamped, Map
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

pub = rospy.Publisher('/marker_trees', Marker, queue_size=10)
pub_lmgt = rospy.Publisher('/marker_trees_gt', Marker, queue_size=10)

pub_lm = rospy.Publisher('/marker_landmarks', Marker, queue_size=10)
pub_mp = rospy.Publisher('/marker_map', Marker, queue_size=10)

def callback(data):

	marker = Marker()
	marker.action = 3
	pub.publish(marker)
	pub_lmgt.publish(marker)

	ids = 0
	for tree in data.trees:

		marker = Marker()
		
		marker.header.frame_id = "sdv";
		marker.header.stamp = data.header.stamp;
		marker.ns = "tree_inferred";
		marker.id = ids; ids += 1;

		marker.type = 3; # CYLINDER
		marker.action = 0;
		
		marker.pose.position.x = tree.distance * math.sin(tree.angle);
		marker.pose.position.y = -tree.distance * math.cos(tree.angle);
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		
		marker.scale.x = tree.diameter;
		marker.scale.y = tree.diameter;
		marker.scale.z = 2;
		marker.color.a = 1.;
		marker.color.r = 1.;
		marker.color.g = 1.;
		marker.color.b = 0.;

		pub.publish( marker );

		marker.ns = "tree_gth";
		marker.header.frame_id = 'gth'
		marker.color.r = 0.
		marker.color.b = 1.

		pub_lmgt.publish(marker)

def callback_landmarks(data):

	marker = Marker()
	marker.action = 3
	pub_lm.publish(marker)

	ids = 0
	for landmark in data.landmarks:

		marker = Marker()
		
		marker.header.frame_id = "world";
		if hasattr(data, 'header'):
			marker.header.stamp = data.header.stamp;
		marker.ns = "landmark";
		marker.id = ids; ids += 1;

		marker.type = 3; # CYLINDER
		marker.action = 0;
		
		marker.pose.position.x = landmark.x;
		marker.pose.position.y = landmark.y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		
		marker.scale.x = landmark.d;
		marker.scale.y = landmark.d;
		marker.scale.z = 2;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;

		pub_lm.publish( marker );


def callback_map(data):

	marker = Marker()
	marker.action = 3
	pub_lm.publish(marker)

	ids = 0
	for landmark in data.landmarks:

		marker = Marker()
		
		marker.header.frame_id = "world";
		if hasattr(data, 'header'):
			marker.header.stamp = data.header.stamp;
		marker.ns = "map";
		marker.id = ids; ids += 1;

		marker.type = 3; # CYLINDER
		marker.action = 0;
		
		marker.pose.position.x = landmark.x;
		marker.pose.position.y = landmark.y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		
		marker.scale.x = landmark.d;
		marker.scale.y = landmark.d;
		marker.scale.z = 5;
		marker.color.a = 0.2;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

		pub_mp.publish( marker );		

import tf

def handle_sdv_pose(msg):
	print 'publishing sdv pose'
	
	br = tf.TransformBroadcaster()
	br.sendTransform(
					(msg.pose.position.x,
					 msg.pose.position.y,
					 msg.pose.position.z),

					(msg.pose.orientation.x,
					 msg.pose.orientation.y,
					 msg.pose.orientation.z,
					 msg.pose.orientation.w),

					msg.header.stamp,
					"sdv",
					"world")


def handle_gth_pose(msg):
	print 'publishing gth pose'
	
	br = tf.TransformBroadcaster()
	br.sendTransform(
					(msg.pose.pose.position.x,
					 msg.pose.pose.position.y,
					 msg.pose.pose.position.z),

					(msg.pose.pose.orientation.x,
					 msg.pose.pose.orientation.y,
					 msg.pose.pose.orientation.z,
					 msg.pose.pose.orientation.w),

					msg.header.stamp,
					"gth",
					"world")

    
def listener():
    rospy.init_node('trees_publisher')
    rospy.Subscriber("/trees", TreesStamped, callback)
    rospy.Subscriber("/landmarks", LandmarksStamped, callback_landmarks)
    rospy.Subscriber("/map", Map, callback_map)
    rospy.Subscriber("/pose", PoseStamped, handle_sdv_pose)
    rospy.Subscriber("/ground_truth_odom", Odometry, handle_gth_pose)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

listener()