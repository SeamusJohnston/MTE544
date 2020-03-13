#include "ekf/ekf.hpp"

EKF::EKF(float freq)
: frequency(freq),
dt_(1.f/freq),
ips_sub_(
        nh_.subscribe("/indoor_pos",
                      1,
                      &EKF::IpsCallback,
                      this)),
odom_sub_(
        nh_.subscribe("/odom",
                      10,
                      &EKF::OdomCallback,
                      this)),
control_sub_(
        nh_.subscribe("/cmd_vel_mux/input/teleop",
                      10,
                      &EKF::ControlCallback,
                      this)),
est_pub_(
        nh_.advertise<geometry_msgs::PoseWithCovariance>(
                      "/pose_est",
                      10, 
                      true)),
gen_(rd_()),
nd_(0, 0.1)    
{
  state_ << 3.07, -0.0138, 1.226;
  state_prev_ = state_;
  P_ << 0.0001, 0, 0, 
        0, 0.0001, 0, 
        0, 0, 0.0001;
  R_ << 0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.01;
  Q_ << 0.1, 0, 0, 0, 0, 
        0, 0.1, 0, 0, 0,
        0, 0, 0.1, 0, 0,
        0, 0, 0, 0.01, 0, 
        0, 0, 0, 0, 0.25;
  // Initialize P, Q, R with config?
}

void EKF::IpsCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  measurement_(int(MEASURE_IDX::x)) = msg->pose.pose.position.x + nd_(gen_);
  measurement_(int(MEASURE_IDX::y)) = msg->pose.pose.position.y + nd_(gen_);
  tf::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  measurement_(int(MEASURE_IDX::theta)) = yaw;
}

void EKF::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  measurement_(int(MEASURE_IDX::v)) = msg->twist.twist.linear.x;
  measurement_(int(MEASURE_IDX::w)) = msg->twist.twist.angular.z;
}

void EKF::ControlCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  control_(int(CONTROL_IDX::v)) = msg->linear.x;
  control_(int(CONTROL_IDX::w)) = msg->angular.z;
}

void EKF::UpdateState() {
  state_prev_ = state_;
  state_(int(STATE_IDX::x)) += control_(int(CONTROL_IDX::v))*cos(state_(int(STATE_IDX::theta)))*dt_;
  state_(int(STATE_IDX::y)) += control_(int(CONTROL_IDX::v))*sin(state_(int(STATE_IDX::theta)))*dt_;
  state_(int(STATE_IDX::theta)) += control_(int(CONTROL_IDX::w))*dt_;
}

void EKF::UpdateMatrices() {
  F_ << 1, 0, -dt_*control_(int(CONTROL_IDX::v))*sin(state_(int(STATE_IDX::theta))),
        0, 1, dt_*control_(int(CONTROL_IDX::v))*cos(state_(int(STATE_IDX::theta))),
        0, 0, 1;

  float d = sqrt(pow(state_(int(STATE_IDX::x)),2) + pow(state_(int(STATE_IDX::y)),2));
  H_ << 1, 0, 0, 
        0, 1, 0,
        state_(int(STATE_IDX::x))/(dt_*d + pow(10,-9)), state_(int(STATE_IDX::y))/(dt_*d + pow(10,-9)), 0, // May want to add back the offsets here
        0, 0, 1,
        0, 0, 1/dt_; // May want to add back the offsets here too
}

void EKF::Update() {
  UpdateState(); // Prediction 
  
  UpdateMatrices(); // Create jacobians

  P_ = F_*P_*F_.transpose() + R_; // Estimate cov
  
  Eigen::Matrix<float, 3, 5> K = P_*H_.transpose()*(H_*P_*H_.transpose() + Q_).inverse(); // Kalman gain

  // Update step
  float d = sqrt(pow(state_(int(STATE_IDX::x)),2) + pow(state_(int(STATE_IDX::y)),2));
  float d_prev = sqrt(pow(state_prev_(int(STATE_IDX::x)),2) + pow(state_prev_(int(STATE_IDX::y)),2));
  Eigen::Matrix<float, 5, 1> h_x; // h(x)
  h_x << state_(int(STATE_IDX::x)), state_(int(STATE_IDX::y)), (d - d_prev)/dt_, state_(int(STATE_IDX::theta)), (state_(int(STATE_IDX::theta)) - state_prev_(int(STATE_IDX::theta)))/dt_;
  state_ += K*(measurement_ - h_x);

  P_ = (Eigen::Matrix3f::Identity() - K*H_)*P_;

  ROS_INFO("State Est: x: %f, y: %f, theta: %f", state_(int(STATE_IDX::x)), state_(int(STATE_IDX::y)), state_(int(STATE_IDX::theta))*180/M_PI);

  Publish();
}

void EKF::Publish() {
  tf::Matrix3x3 m;
  tf::Quaternion q;
  m.setRPY(0, 0, state_(int(STATE_IDX::theta)));
  m.getRotation(q);
  geometry_msgs::PoseWithCovariance msg;
  msg.pose.position.x = state_(int(STATE_IDX::x));
  msg.pose.position.y = state_(int(STATE_IDX::y));
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  msg.covariance = {P_(0,0), P_(0,1), 0, 0, 0, P_(0,2),
                    P_(1,0), P_(1,1), 0, 0, 0, P_(1,2),
                    0, 0, pow(10,5), 0, 0, 0, 
                    0, 0, 0, pow(10,5), 0, 0,
                    0, 0, 0, 0, pow(10,5), 0,
                    P_(2,0), P_(2,1), 0, 0, 0, P_(2,2)};
  est_pub_.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ekf_node");
  EKF ekf(10.0);
  ros::Rate loop_rate(10.0);
  while (ros::ok())
  {
    ekf.Update();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}