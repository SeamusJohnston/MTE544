// Mapping Node Implementation
#include <turtlebot_example/mapper_node.h>


Mapping::Mapping() {
  Cell default_cell;
  width_ = 2000;
  height_ = 2000;
  x_offset_ = width_ / 2;
  y_offset_ = height_ / 2;

  vector<vector<Cell>> temp_map(width_, vector<Cell>(height_, default_cell));

  pose_sub_ = nh_.subscribe("/gazebo/model_states", 1, &Mapping::PoseCallback, this);
  scan_sub_ = nh_.subscribe("/scan", 1, &Mapping::ScanCallback, this);
  position_sub_ = nh_.subscribe("/indoor_pos", 1, &Mapping::IndoorPositionCallback, this);

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, false);

  for (int row = 0; row < temp_map.size(); row ++) {
    for (int col = 0; col < temp_map[row].size(); col++) {
      temp_map[row][col].log_odds = log_odds(unknown_constant_);
    }
  }
  map_ = temp_map;
}

void Mapping::PrintMap(vector<vector<Cell>> map) {
  string map_debug_string = "\n";
  for (int row = 0; row < map.size(); row ++) {
    for (int col = 0; col < map[row].size(); col++) {
      map_debug_string += to_string(map[row][col].log_odds) + ", ";
    }
    map_debug_string += "\n";
  }

  ROS_INFO("Map: %s", map_debug_string.c_str());
}

double Mapping::log_odds(double prob) {
  return log(prob / (1 - prob));
}

void Mapping::convertToOccupancyFrame(double world_x, double world_y, int& occ_x, int& occ_y) {
  double multiplier = 50;
  occ_x = (world_x * multiplier + x_offset_);
  occ_y = (world_y * multiplier + y_offset_);
}


void Mapping::ScanCallback(const sensor_msgs::LaserScan& msg) {
  if (!position_set_) {
    ROS_WARN("Position not yet set, not using this scan data until position i set");
    return;
  }
  position_set_ = false;

  iteration_ ++;
  if (iteration_ == 3) {
    iteration_ = 0;
    return;
  }
  double increment = msg.angle_increment;

  for (int i = 0; i < msg.ranges.size(); i++) {

    if (i % 2 != 0) {
      continue;
    } else {

      double angle = msg.angle_min + msg.angle_increment * i;
      double range = msg.ranges[i];

      if (!isnan(range) && range > msg.range_min && range < msg.range_max) {
        geometry_msgs::Pose scan_pose;
        double x = range * sin(angle);
        double y = range * cos(angle);
        scan_pose.position.x = x;
        scan_pose.position.y = y;

        tf::Transform robot_to_scan;
        tf::poseMsgToTF(scan_pose, robot_to_scan);

        tf::Transform world_to_scan = world_to_robot_ * robot_to_scan;
        geometry_msgs::Pose world_to_scan_pose;
        tf::poseTFToMsg(world_to_scan, world_to_scan_pose);

        int occupancy_to_scan_x;
        int occupancy_to_scan_y;
        convertToOccupancyFrame(world_to_scan_pose.position.x,
                                world_to_scan_pose.position.y,
                                occupancy_to_scan_x, occupancy_to_scan_y);
        // ROS_INFO("    World->Scan(%f, %f) Mapped Occupancy(%f, %f)",
        //   world_to_scan_pose.position.x, world_to_scan_pose.position.y,
        //   occupancy_to_scan_x, occupancy_to_scan_y);

        int occupancy_to_robot_x;
        int occupancy_to_robot_y;
        convertToOccupancyFrame(world_to_robot_.getOrigin().getX(),
                                world_to_robot_.getOrigin().getY(),
                                occupancy_to_robot_x, occupancy_to_robot_y);
        // ROS_INFO("      World->Robot(%f, %f) Mapped Occupancy(%f, %f)",
        //   world_to_robot_.getOrigin().getX(), world_to_robot_.getOrigin().getY(),
        //   occupancy_to_robot_x, occupancy_to_robot_y);

        std::vector<Coordinate> coordinates_to_clear =
          findAllPointsAlongLine(occupancy_to_robot_x, occupancy_to_robot_y,
                                 occupancy_to_scan_x, occupancy_to_scan_y);

        for (int j = 0; j < coordinates_to_clear.size(); j++) {
          double probability = free_constant_;

          if (j == coordinates_to_clear.size() -1) {
            probability = occupied_constant_;
          } else {
            probability = free_constant_;
          }

          Cell& cell = map_[coordinates_to_clear[j].x][coordinates_to_clear[j].y];
          double old_cell_log_odds = cell.log_odds;
          cell.log_odds = cell.log_odds + log_odds(probability) - log_odds(unknown_constant_);
        }
      }
    }
  }
}

void Mapping::PoseCallback(const gazebo_msgs::ModelStates& msg) {
  position_set_ = true;
  if (msg.pose.size() == 0)
    return;
  pose_ = msg.pose[0];

  pose_.position.z = 0;

  tf::poseMsgToTF(pose_, world_to_robot_);
  PublishMap();
}

void Mapping::IndoorPositionCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  position_set_ = true;
  pose_ = msg.pose.pose;
  pose_.position.z = 0;
  tf::poseMsgToTF(pose_, world_to_robot_);
  PublishMap();
}

void Mapping::OdomPositionCallback(const nav_msgs::Odometry& msg) {
  position_set_ = true;
  pose_ = msg.pose.pose;
  pose_.position.z = 0;
  tf::poseMsgToTF(pose_, world_to_robot_);
  PublishMap();
}

std::vector<Coordinate> Mapping::findAllPointsAlongLine(int x1, int y1, int x2, int y2) {
    std::vector<Coordinate> ptsAlongLine;

    int slope_x1 = 0;
    int slope_y1 = 0;
    int slope_x2 = 0;
    int slope_y2 = 0;

    int width = x2 - x1;
    int height = y2 - y1;
    int xTracker = x1;
    int yTracker = y1;

    if (width < 0) {
        slope_x1 = -1;
        slope_x2 = -1;
    } else if (width > 0) {
        slope_x1 = 1;
        slope_x2 = 1;
    }

    if (height < 0) {
        slope_y1 = -1;
    } else if (height > 0) {
        slope_y1 = 1;
    }

    int longest;
    int shortest;

    if (!(abs(width) > abs(height))) {
        longest = abs(height);
        shortest = abs(width);
        if (height < 0) {
            slope_y2 = -1;
        } else if (height > 0) {
            slope_y2 = 1;
        }
        slope_x2 = 0;
    } else {
        longest = abs(width);
        shortest = abs(height);
    }

    int numerator = longest >> 1;

    for (int i=0; i<=longest; i++) {
        Coordinate c;
        c.x = xTracker;
        c.y = yTracker;
        ptsAlongLine.push_back(c);

        numerator += shortest;

        if (!(numerator < longest)) {
            numerator -= longest;
            xTracker += slope_x1;
            yTracker += slope_y1;
        } else {
            xTracker += slope_x2;
            yTracker += slope_y2;
        }
    }

    return ptsAlongLine;
}

void Mapping::PublishMap() {
  nav_msgs::OccupancyGrid occ_grid;
  occ_grid.info.resolution = 1.0;
  occ_grid.info.width = width_;
  occ_grid.info.height = height_;

  vector<signed char> data_map;
  for (int i = 0; i < map_.size(); i ++) {
    for (int j = 0; j < map_[i].size(); j++) {
        Cell cell = map_[i][j];
        double probability = (1 - 1/(1 + exp(cell.log_odds))) * 100;
        data_map.push_back(static_cast<int>(probability));
    }
  }
  occ_grid.data = data_map;

  ROS_WARN("Publishing Occ Grid");
  map_pub_.publish(occ_grid);
}
