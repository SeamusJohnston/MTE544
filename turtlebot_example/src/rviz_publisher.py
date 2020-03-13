#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

path_ips_pub = rospy.Publisher('ips_path', Path, queue_size=10)
path_ekf_pub = rospy.Publisher('path_ekf', Path, queue_size=10)
viz_ekf_pub = rospy.Publisher('ekf_viz', Marker, queue_size=10)

path_ips = Path()
path_ekf = Path()

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

  # Publish visualization ellipses
  marker = Marker()
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
  marker
  viz_ekf_pub.publish(marker)

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

if __name__ == '__main__':
    rospy.init_node('rviz_publisher')
    rospy.Subscriber('/pose_est',
                     PoseWithCovariance,
                     EKFCallback)
    rospy.Subscriber('/indoor_pos',
                     PoseWithCovarianceStamped,
                     IPSCallback)          
    rospy.spin()
