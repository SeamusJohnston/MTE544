#pragma once
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <Eigen/Dense>
#include <random>

class EKF {
 public:
  EKF(float freq);
  void Update();
  float frequency;
  
  enum class STATE_IDX{
    x,
    y,
    theta
  };
  enum class CONTROL_IDX{
    v,
    w
  };
  enum class MEASURE_IDX{
    x,
    y, 
    v,
    theta,
    w
  };

 private:
  ros::NodeHandle nh_;
  float dt_;
  ros::Subscriber ips_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber control_sub_;
  ros::Publisher est_pub_;
  Eigen::Vector3f state_;
  Eigen::Vector3f state_prev_;
  Eigen::Vector2f control_;
  Eigen::Matrix<float, 5, 1> measurement_;
  Eigen::Matrix3f F_; // State Transition
  Eigen::Matrix<float, 5, 5> Q_; // Measurement Cov
  Eigen::Matrix3f R_; // Process Cov
  Eigen::Matrix<float, 5, 3> H_; // Observation
  Eigen::Matrix3f P_; // Estimate Cov
  std::random_device rd_;
  std::mt19937 gen_;
  std::normal_distribution<float> nd_;

  void UpdateState(); // f(x,u)
  void UpdateMatrices(); // Linearize A and H
  void Publish();
  void IpsCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void ControlCallback(const geometry_msgs::Twist::ConstPtr& msg);

};
