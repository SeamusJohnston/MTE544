#ifndef MAPPER_NODE_H
#define MAPPER_NODE_H

#include <ros/console.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "ros/ros.h"

using namespace std;

enum State {
  UNKNOWN=0,
  OCCUPIED=1,
  FREE=2
};

struct Cell {
  double log_odds = 0;
  State state = UNKNOWN;
};

struct Coordinate {
    int x;
    int y;
};

class Mapping {
private:
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber position_sub_;
  ros::Publisher map_pub_;
  ros::Subscriber odom_pose_sub_;

  int iteration_ = 0;
  double x_offset_;
  double y_offset_;

  double free_constant_ = 0.97;
  double unknown_constant_ = 0.98;
  double occupied_constant_ = 0.99;

  bool position_set_ = false;
  geometry_msgs::Pose pose_;
  tf::Transform world_to_robot_;

  int width_;
  int height_;
  vector<vector<Cell>> map_;

public:
  Mapping();
  void CreateMap();
  void PoseCallback(const gazebo_msgs::ModelStates& msg);
  void PrintMap(vector<vector<Cell>> map);
  void ScanCallback(const sensor_msgs::LaserScan& msg);
  void IndoorPositionCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
  std::vector<Coordinate> findAllPointsAlongLine(int x1, int y1, int x2, int y2);
  void PublishMap();
  double log_odds(double prob);
  void convertToOccupancyFrame(double world_x, double world_y, int& occ_x, int& occ_y);
  void OdomPositionCallback(const nav_msgs::Odometry& msg);
};

#endif
