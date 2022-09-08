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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
// #include <eigen3/Eigen/src/Core/Matrix.h>
// #include "eigen3/Eigen/src/Geometry"
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
using Eigen::Matrix2f;
using Eigen::Rotation2Df;
using Eigen::Affine2f;
#include "vector_map/vector_map.h"
#include "shared/math/statistics.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::Line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

Eigen::Affine2f prev_transform;
Vector2f prev_odom_pose;
float prev_odom_angle;

float upd_dist = 0.0;
float upd_angle = 0.0;
float x_tr_tr_err = 0.15;
float y_tr_tr_err = 0.03;
float tr_rot_err = 0.06;
float rot_rot_err = 0.15;
float rot_tr_err = 0.15;
float lidar_stddev = 0.075;
float gamma = 0.15;
float d_short = 0.325;
float d_long = 1.15; 
int update_ct = 0;


ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {
      prev_odom_angle = 0;
      prev_odom_pose = Vector2f(0, 0);
    }

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);

  angle_min += angle;
  angle_max += angle;

  float curr_angle = angle_min;

  for (int range = 0; range < num_ranges; range += 15) {
    Vector2f low = (geometry::Heading(curr_angle) * range_min) + loc;
    Vector2f up = (geometry::Heading(curr_angle) * range_max) + loc;
    Line2f lidar = Line2f(low, up);
    float closest_dist = std::numeric_limits<float>::max();
    Vector2f closest_pt(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    
    for (size_t i = 0; i < map_.lines.size(); i++) {
      
      const Line2f map_line = map_.lines[i];
      // The Line2f class has helper functions that will be useful.
      // You can create a new line segment instance as follows, for :
      // Access the end points using `.p0` and `.p1` members:
      // printf("P0: %f, %f P1: %f,%f\n", 
      //       map_line.p0.x(),
      //       map_line.p0.y(),
      //       map_line.p1.x(),
      //       map_line.p1.y());

      // Check for intersections:
      // You can also simultaneously check for intersection, and return the point
      // of intersection:
      Vector2f intersection_point; // Return variable
      bool intersects = lidar.Intersection(map_line, &intersection_point);
      if (intersects) {
        float distance = (intersection_point - loc).squaredNorm();
        if (distance < closest_dist) {
          closest_dist = distance;
          closest_pt = intersection_point;
        }
        printf("Intersects at %f,%f\n", 
              intersection_point.x(),
              intersection_point.y());
      } else {
        // printf("No intersection\n");
      }
      
    }

    scan[range] = closest_pt;
    curr_angle += ((angle_max - angle_min) / num_ranges) * 15;

  }

  // // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  // for (size_t i = 0; i < scan.size(); ++i) {
  //   scan[i] = Vector2f(0, 0);
  // }

  // // The line segments in the map are stored in the `map_.lines` variable. You
  // // can iterate through them as:
  // for (size_t i = 0; i < map_.lines.size(); ++i) {
  //   /*
  //   const Line2f map_line = map_.lines[i];
  //   // The Line2f class has helper functions that will be useful.
  //   // You can create a new line segment instance as follows, for :
  //   Line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
  //   // Access the end points using `.p0` and `.p1` members:
  //   printf("P0: %f, %f P1: %f,%f\n", 
  //          my_line.p0.x(),
  //          my_line.p0.y(),
  //          my_line.p1.x(),
  //          my_line.p1.y());

  //   // Check for intersections:
  //   bool intersects = map_line.Intersects(my_line);
  //   // You can also simultaneously check for intersection, and return the point
  //   // of intersection:
  //   Vector2f intersection_point; // Return variable
  //   intersects = map_line.Intersection(my_line, &intersection_point);
  //   if (intersects) {
  //     printf("Intersects at %f,%f\n", 
  //            intersection_point.x(),
  //            intersection_point.y());
  //   } else {
  //     printf("No intersection\n");
  //   }
  //   */
  // }
}

vector<double> get_likelihoods(const vector<float>& ranges, vector<Vector2f> scan, Vector2f laser_part, float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max) {
  vector<double> likelihoods;
  int num_ranges = ranges.size();
  for (int i = 0; i < num_ranges; i += 15) {
    float dist;
    if (scan[i] != Vector2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max())) {
      dist = (scan[i] - laser_part).norm();
    } else {
      dist = 1e9;
    }
    float lidar_distance = ranges[i];

    if (lidar_distance <= range_min || lidar_distance >= range_max) {
      likelihoods.push_back(0);
    } else if (lidar_distance <= dist - d_short) {
      double obs_likelihood = statistics::ProbabilityDensityGaussian<double>(
          d_short, 0.0, lidar_stddev);
      likelihoods.push_back(obs_likelihood);
    } else if (dist + d_long <= lidar_distance) {
      double obs_likelihood = statistics::ProbabilityDensityGaussian<double>(
          d_long, 0.0, lidar_stddev);
      likelihoods.push_back(obs_likelihood);
    } else {
      double obs_likelihood = statistics::ProbabilityDensityGaussian<double>(
          dist, lidar_distance, lidar_stddev);
      likelihoods.push_back(obs_likelihood);
    }
  }

  return likelihoods;
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.

  Vector2f laser_loc(0.2, 0);
  Particle particle = *p_ptr;
  Vector2f laser_part = particle.loc + laser_loc;
  vector<Vector2f> scan; 
  GetPredictedPointCloud(laser_part, particle.angle, ranges.size(), range_min, range_max, angle_min, angle_max, &scan);

  vector<double> likelihoods = get_likelihoods(ranges, scan, laser_part, range_min, range_max, angle_min, angle_max);
  double total_log_likelihood = 0;
  for (unsigned int i = 0; i < likelihoods.size(); i++) {
    if (likelihoods[i] != 0) {
      double log_likelihood = std::log(likelihoods[i]);
      total_log_likelihood += log_likelihood;
    }
  }
  total_log_likelihood *= gamma;
  p_ptr->weight = total_log_likelihood;
}



void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable.
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  // float x = rng_.UniformRandom(0, 1);
  // printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
  //        x);

  vector<Particle> old_particles;
  GetParticles(&old_particles);
  vector<Particle> new_particles;

  double max_weight = -1e100;
  int num_parts = old_particles.size();
  for (int i = 0; i < num_parts; i++) {
    max_weight = std::max(max_weight, old_particles[i].weight);
  }

  // Normalize particle weights
  for (int i = 0; i < num_parts; i++) {
    old_particles[i].weight = old_particles[i].weight - max_weight;
  }

  vector<double> cumul_prob(old_particles.size());
  cumul_prob[0] = std::exp(old_particles[0].weight);

  for (int i = 1; i < num_parts; i++) {
    double prob = std::exp(old_particles[i].weight);
    cumul_prob[i] = cumul_prob[i - 1] + prob; 
  }
  double total_prob = cumul_prob[old_particles.size() - 1];
  double curr_rand = rng_.UniformRandom(0, total_prob);
  double step_size = total_prob / old_particles.size();

  for (int i = 0; i < num_parts; i++) {
    unsigned int elem;
    for(elem = 0; elem < cumul_prob.size(); ++elem) {
      if (cumul_prob[elem] >= curr_rand) {
        break;
      }
    }

    new_particles.push_back(old_particles[elem]);

    curr_rand = curr_rand + step_size;
    curr_rand = std::fmod(curr_rand, total_prob);
  }

  particles_ = new_particles;

}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  float upd_min_dist = 0.05;
  float upd_min_angle = 0.0873; 

  if (upd_dist > upd_min_dist ||
      upd_angle > upd_min_angle) {

    upd_dist = 0.0;
    upd_angle = 0.0;

    for (unsigned int i = 0; i < particles_.size(); i++) {
      Update(ranges, range_min, range_max, angle_min, angle_max, &particles_[i]);
    }
    update_ct++;

    if (update_ct % 3 == 0) {
      update_ct = 0;
      Resample();
    }
  }
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

   if (!odom_initialized_) {
    odom_initialized_ = true;
    prev_odom_angle = odom_angle;
    prev_odom_pose = odom_loc;
    return;
  }

  float delta_theta = math_util::AngleDiff(odom_angle, prev_odom_angle);
  upd_angle += std::abs(delta_theta);
  Vector2f trans_diff = PerformCoordTransform(odom_angle, odom_loc);
  // Eigen::Vector2f a_t2_map_minus_t1_map = PerformCoordTransform(odom_angle, odom_loc);
  // float d_x = std::abs(a_t2_map_minus_t1_map.x());
  // float d_y = std::abs(a_t2_map_minus_t1_map.y());

  // float delta_dist = (odom_loc - prev_odom_pose).norm();
  int num_parts = particles_.size();
  for(int i = 0; i < num_parts; i++) {
    Particle particle = particles_[i];

    // particle.loc += a_t2_map_minus_t1_map;
    // particle.angle += delta_theta;

    // float e_x = rng_.Gaussian(0.0, tr_tr_err * d_x + tr_rot_err * delta_theta);
    // float e_y = rng_.Gaussian(0.0, tr_tr_err * d_y + tr_rot_err * delta_theta);
    // float e_theta = rng_.Gaussian(0.0, rot_tr_err * delta_dist + rot_rot_err * delta_theta);

    // particle.loc = Vector2f(particle.loc.x() + e_x, particle.loc.y() + e_y);
    // particle.angle += e_theta;

    Eigen::Rotation2Df map_rotation(particle.angle);
    Vector2f map_trans_diff = map_rotation * trans_diff;

    float e_x = rng_.Gaussian(0.0, x_tr_tr_err * trans_diff.norm() + tr_rot_err * std::abs(delta_theta));
    float e_y = rng_.Gaussian(0.0, y_tr_tr_err * trans_diff.norm() + tr_rot_err * std::abs(delta_theta));
    float e_theta = rng_.Gaussian(0.0, rot_tr_err * trans_diff.norm() + rot_rot_err * std::abs(delta_theta));
    particle.loc = particle.loc + map_trans_diff + (map_rotation * Vector2f(e_x, e_y));
    particle.angle = particle.angle + delta_theta + e_theta;

    particles_[i] = particle;
  }

  prev_odom_angle = odom_angle;
  prev_odom_pose = odom_loc;

}

Eigen::Vector2f ParticleFilter::PerformCoordTransform(const float odom_angle, const Vector2f& odom_loc) {
  Vector2f transl_diff_rel_odom = odom_loc - prev_odom_pose;
  Eigen::Rotation2Df inv_rot_rel_t1(-prev_odom_angle);
  Vector2f transl_diff_rel_t1 = inv_rot_rel_t1 * transl_diff_rel_odom;
  upd_dist += transl_diff_rel_t1.norm();

  return transl_diff_rel_t1;


  // float delta_theta = odom_angle - prev_odom_angle;

  // Eigen::Rotation2Df t1_rotation(prev_odom_angle);
  // Eigen::Translation2f t1_translation = Eigen::Translation2f(prev_odom_pose);
  // Eigen::Affine2f a_t1 = t1_translation * t1_rotation;
  // Eigen::Affine2f a_t1_inv = a_t1.inverse();

  // Eigen::Rotation2Df t2_rotation(odom_angle);
  // Eigen::Translation2f t2_translation = Eigen::Translation2f(odom_loc);
  // Eigen::Affine2f a_t2 = t2_translation * t2_rotation;

  // Eigen::Affine2f a_robot_t1_t2 = a_t1_inv * a_t2;

  // Eigen::Affine2f a_t2_map = prev_transform * a_robot_t1_t2;
  // Eigen::Vector2f a_t2_map_minus_t1_map = a_t2_map.translation() - prev_transform.translation();
  // prev_transform = a_t2_map;

  // return a_t2_map_minus_t1_map;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  particles_.clear();

  Eigen::Rotation2Df map_rotation(angle);
  Eigen::Translation2f map_translation(loc);

  map_.Load(map_file);

  for (int i = 0; i < FLAGS_num_particles; i++) {
    particles_.push_back(Particle(loc, angle, 0));
  }
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;

  loc = Vector2f(0, 0);
  float sin_avg = 0.0;
  float cos_avg = 0.0;
  for (int i = 0; i < FLAGS_num_particles; i++) {
    loc += particles_[i].loc;
    sin_avg += std::sin(particles_[i].angle);
    cos_avg += std::cos(particles_[i].angle);
  }
  loc /= FLAGS_num_particles;
  sin_avg /= FLAGS_num_particles;
  cos_avg /= FLAGS_num_particles;

  if (sin_avg == 0.0 && cos_avg == 0.0) {
    angle = 0.0;
  } else {
    angle = std::atan2(sin_avg, cos_avg);
  }
}


}  // namespace particle_filter
