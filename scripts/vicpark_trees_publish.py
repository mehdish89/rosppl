import rospy
from std_msgs.msg import String
from vicpark_msgs.msg import TreesStamped, Tree
from vicpark_msgs.msg import LandmarksStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import math

pub = rospy.Publisher('/marker_trees', Marker, queue_size=10)
pub_lm = rospy.Publisher('/marker_landmarks', Marker, queue_size=10)

def callback(data):

	marker = Marker()
	marker.action = 3
	pub.publish(marker)

	ids = 0
	for tree in data.trees:

		marker = Marker()
		
		marker.header.frame_id = "sdv";
		marker.header.stamp = data.header.stamp;
		marker.ns = "tree";
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
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;

		pub.publish( marker );

def callback_landmarks(data):

	marker = Marker()
	marker.action = 3
	pub_lm.publish(marker)

	ids = 0
	for landmark in data.landmarks:

		marker = Marker()
		
		marker.header.frame_id = "world";
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

import tf

def handle_sdv_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.position.x,
    				  msg.pose.position.y,
    				  msg.pose.position.z),

                     (msg.pose.orientation.x,
                      msg.pose.orientation.y,
                      msg.pose.orientation.z,
                      msg.pose.orientation.w),

                     msg.header.stamp,
                     "sdv",
                     "world")
    
def listener():
    rospy.init_node('trees_publisher')
    rospy.Subscriber("/trees", TreesStamped, callback)
    rospy.Subscriber("/landmarks", LandmarksStamped, callback_landmarks)
    rospy.Subscriber("/pose", PoseStamped, handle_sdv_pose)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

listener()