//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "stdio.h"
#include <cmath>
#include "sensor_msgs/LaserScan.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

DEFINE_double(cp2_distance, 2.5, "Distance to travel (cp2)");
// DEFINE_double(cp_curvature, 0.5, "Curvature for arc path (cp1)");

DEFINE_double(cp2_curvature, 0.5, "Curvature for arc path (cp2)");
DEFINE_double(car_length, 0.4, "Distance between the base_link frame origin and the front of the car");


namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

navigation::NavConsts nav_consts_;
navigation::NavTimestepData prev_nav_data_;
navigation::NavTimestepData curr_nav_data;
int frequency = 20;
const float car_width = 0.14;
const float wheel_base = 0.4;
float latency = 0.2;
} //namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);

  nav_consts_.v_max = 1.0;
  nav_consts_.a_max = 3.0;
  nav_consts_.d_max = 3.0;

  curr_nav_data.stopping_dist = (nav_consts_.v_max * nav_consts_.v_max) / (2 * nav_consts_.d_max);
  curr_nav_data.dist_remaining = FLAGS_cp2_distance;
  curr_nav_data.cur_v = 0;
  curr_nav_data.cur_curv = FLAGS_cp2_curvature;
  if (FLAGS_cp2_curvature > 0.01) {
    latency = 0.15;
  }
	
  collision_pt = Vector2f(0, 0);

}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    curr_nav_data.odom_loc = loc;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;

  prev_nav_data_ = curr_nav_data;
  float prev_vel = prev_nav_data_.cur_v;

  curr_nav_data.odom_loc = loc;

  float delta_distance = (curr_nav_data.odom_loc - prev_nav_data_.odom_loc).norm();
  float compensated_distance = curr_nav_data.dist_remaining - delta_distance - prev_vel * latency;
  curr_nav_data.dist_remaining = std::max(compensated_distance, 0.0f);

  // curr_nav_data.dist_remaining -= (curr_nav_data.odom_loc - prev_nav_data_.odom_loc).norm();

}

void Navigation::OptimalControl() {
  if (curr_nav_data.dist_remaining > curr_nav_data.stopping_dist) {
    if (curr_nav_data.cur_v < nav_consts_.v_max) {
      curr_nav_data.cur_v += nav_consts_.a_max / frequency;
    } else if (curr_nav_data.cur_v == nav_consts_.v_max) {
      // do nothing
    } else {
      // we should not be here
    }
  } 
  // we need to stop
  else if (curr_nav_data.dist_remaining > 0) {
    curr_nav_data.cur_v -= nav_consts_.d_max / frequency;
  } else {
    curr_nav_data.cur_v = 0;
  }

  // set velocity
  drive_msg_.velocity = curr_nav_data.cur_v;
  drive_msg_.curvature = curr_nav_data.cur_curv;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

void Navigation::LaserToPtCloud(const sensor_msgs::LaserScan &msg,
                             const Vector2f &laserLoc,
                             vector<Vector2f> *point_cloud_) {
  int num_pts = msg.ranges.size();
  float curr_angle = msg.angle_min;

  for (int i = 0; i < num_pts; i++) {
    float curr_radius = msg.ranges[i];
    if (curr_radius < msg.range_min || curr_radius > msg.range_max) {
      continue;
    }

    Vector2f new_pt;
    new_pt[0] = curr_radius * std::cos(curr_angle);
    new_pt[1] = curr_radius * std::sin(curr_angle);
    new_pt += laserLoc;
    point_cloud_->push_back(new_pt);

    curr_angle += msg.angle_increment;
  }
}

vector<Vector2f> Navigation::DetectCollisionPoints(float curvature) {
  const float MIN_TURNING_CURVATURE = 0.01;
  const float max_y = obstacle_threshold_ + car_width;

  float radius;
  float r1;
  float r2;
  Vector2f center;
  if (abs(curvature) >= MIN_TURNING_CURVATURE) {
    radius = 1 / curvature;
    r1 = (abs(radius)) - (max_y);
    r2 = std::sqrt(pow((abs(radius) + max_y), 2) +
                   pow((FLAGS_car_length + obstacle_threshold_), 2));
    center = Vector2f(0, radius);
  }

  vector<Vector2f> collision_pts;
  int pt_cloud_size = point_cloud_.size();
  for (int i = 0; i < pt_cloud_size; i++) {
    Vector2f point = point_cloud_[i];
    if (abs(curvature) < MIN_TURNING_CURVATURE) {
      if (abs(point.y()) <= max_y) {
        collision_pts.push_back(point);
      }
    } else {
      float distance = (point - center).norm();
      // float distance = calculate_distance(point, center);
      if (distance >= r1 && distance <= r2) {
        collision_pts.push_back(point);
      }
    }
  }

  return collision_pts;
}

vector<float> Navigation::GetFreePathLength(std::vector<Eigen::Vector2f> *points, float curvature) {
  const float MIN_TURNING_CURVATURE = 0.01;
  vector<float> path_lens;
  int num_pts = points->size();

  for (int i = 0; i < num_pts; i++) {
    Vector2f point = points->at(i);
    if (abs(curvature) < MIN_TURNING_CURVATURE) {
      // straight line
      path_lens.push_back(point.x() - wheel_base - obstacle_threshold_);
    } else {
      float r = 1 / curvature;
      float theta = atan2(point.x(), r - point.y());

      float h = FLAGS_car_length + obstacle_threshold_;
      float w = car_width + obstacle_threshold_;
      float omega = atan2(h, r - w);
      float phi = theta - omega;

      path_lens.push_back(r * phi);
    }
  }

  return path_lens;
  
}

void Navigation::AvoidObstacles(const sensor_msgs::LaserScan &msg) {
  const Vector2f kLaserLoc(0.2, 0);

  static vector<Vector2f> point_cloud_;
  // TODO Convert the LaserScan to a point cloud
  // The LaserScan parameters are accessible as follows:
  // msg.angle_increment // Angular increment between subsequent rays
  // msg.angle_max // Angle of the first ray
  // msg.angle_min // Angle of the last ray
  // msg.range_max // Maximum observable range
  // msg.range_min // Minimum observable range
  // msg.ranges[i] // The range of the i'th ray
  LaserToPtCloud(msg, kLaserLoc, &point_cloud_);
  ObservePointCloud(point_cloud_, msg.header.stamp.toSec());
  vector<Eigen::Vector2f> collision_pts = DetectCollisionPoints(FLAGS_cp2_curvature);
  vector<float> path_lens = GetFreePathLength(&collision_pts, FLAGS_cp2_curvature);

  float min_free_path_length = FLT_MAX;
  Vector2f closest_pt;
  for (unsigned int i = 0; i < collision_pts.size(); i++) {
    float distance = path_lens[i];
    if (distance >= 0) {
      min_free_path_length = std::min(min_free_path_length, distance);
      closest_pt = collision_pts[i];
    }
  }
  float free_angle = FLT_MAX;
  float free_angle_length = FLT_MIN;
  Vector2f free_angle_pt;

  for (float angle = -1; angle < 1; angle += 0.1) {
    vector<Eigen::Vector2f> collision_pts = DetectCollisionPoints(angle);
    vector<float> path_lens = GetFreePathLength(&collision_pts, angle);

    float min_free_path_length = FLT_MAX;
    Vector2f closest_pt;
    for (unsigned int i = 0; i < collision_pts.size(); i++) {
      float dist = path_lens[i];
      if (dist >= 0 && dist < min_free_path_length) {
         min_free_path_length = dist;
         closest_pt = collision_pts[i];
      }
    }

    if ((min_free_path_length > free_angle_length) ||
        ((min_free_path_length == free_angle_length) &&
         (abs(angle - FLAGS_cp2_curvature) < abs(free_angle - FLAGS_cp2_curvature)))) {
      free_angle_length = min_free_path_length;
      free_angle = angle;
      free_angle_pt = closest_pt;
    }
  }

  // Let's see which angle to use...
  const float CHANGE_ANGLE_THRESH = 4.0;
  if (min_free_path_length < CHANGE_ANGLE_THRESH) {
    // Use a different angle from the desired curvature
    curr_nav_data.cur_curv = free_angle; 
    SetDistRemaining(free_angle_length);
    if (free_angle_length != FLT_MAX) {
      collision_pt = free_angle_pt;
    } else {
      collision_pt = Vector2f(0, 0);
    }
  } else {
    curr_nav_data.cur_curv = FLAGS_cp2_curvature; 
    SetDistRemaining(min_free_path_length);
    if (min_free_path_length!= FLT_MAX) {
      collision_pt = closest_pt;
    } else {
      collision_pt = Vector2f(0, 0);
    }
  }
}

void Navigation::SetDistRemaining(const float dist) {
  curr_nav_data.dist_remaining = std::min(curr_nav_data.dist_remaining, dist);
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"
  OptimalControl();
  visualization::DrawCross(collision_pt, 0.3, 0xff0000, local_viz_msg_);

  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = ...;
  // drive_msg_.velocity = ...;

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
