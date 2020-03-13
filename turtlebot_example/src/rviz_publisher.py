#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

path_ips_pub = rospy.Publisher('ips_path', Path, queue_size=10)
path_ips_noise_pub = rospy.Publisher('ips_noise_path', Path, queue_size=10)
path_ekf_pub = rospy.Publisher('path_ekf', Path, queue_size=10)
viz_ekf_pub = rospy.Publisher('ekf_viz', MarkerArray, queue_size=10)

path_ips = Path()
path_ekf = Path()
path_ips_noise = Path()
marker_list = MarkerArray()
counter = 0
marker_count = 0

# TODO: Not sure if the paths/markers need frame info in order for rviz to work correctly

def EKFCallback(msg):
  global path_ekf
  # Publish path (add most recent pose onto it)
  path_ekf.header.frame_id = "map"
  pose_new = PoseStamped()
  pose_new.header.frame_id = "map"
  pose_new.pose = msg.pose
  path_ekf.poses.append(pose_new)
  path_ekf_pub.publish(path_ekf)

  global marker_list
  global counter
  global marker_count
  if counter == 10:
    # Publish visualization ellipses
    marker = Marker()
    marker.id = marker_count
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "map"
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position = msg.pose.position
    marker.pose.orientation.w = 1
    marker.scale.x = msg.covariance[0]
    marker.scale.y = msg.covariance[7]
    marker.scale.z = 0.001
    marker.color.a = 1
    marker_list.markers.append(marker)
    viz_ekf_pub.publish(marker_list)
    counter = 0
    marker_count = marker_count + 1
  counter = counter + 1

def IPSCallback(msg):
  global path_ips
  # Publish path (add most recent pose onto it)
  path_ips.header.frame_id = "map"
  pose_new = PoseStamped()
  pose_new.header.frame_id = "map"
  pose_new.pose = msg.pose.pose
  pose_new.pose.position.z = 0
  path_ips.poses.append(pose_new)
  path_ips_pub.publish(path_ips)

def IPSNoiseCallback(msg):
  global path_ips_noise
  # Publish path (add most recent pose onto it)
  path_ips_noise.header.frame_id = "map"
  pose_new = PoseStamped()
  pose_new.header.frame_id = "map"
  pose_new.pose = msg.pose.pose
  pose_new.pose.position.z = 0
  path_ips_noise.poses.append(pose_new)
  path_ips_noise_pub.publish(path_ips_noise)

if __name__ == '__main__':
    rospy.init_node('rviz_publisher')
    rospy.Subscriber('/pose_est',
                     PoseWithCovariance,
                     EKFCallback)
    rospy.Subscriber('/indoor_pos',
                     PoseWithCovarianceStamped,
                     IPSCallback)
    rospy.Subscriber('/indoor_pos_noise',
                     PoseWithCovarianceStamped,
                     IPSNoiseCallback)             
    rospy.spin()
