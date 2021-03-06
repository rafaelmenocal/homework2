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

#include "particle_filter.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "config_reader/config_reader.h"
#include "ros/ros.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "vector_map/vector_map.h"

using geometry::line2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 150, "Number of particles");
double current_time;
const Vector2f kLaserLoc(0.2, 0);

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});
CONFIG_FLOAT(k1, "k1");
CONFIG_FLOAT(k2, "k2");
CONFIG_FLOAT(k3, "k3");
CONFIG_FLOAT(k4, "k4");
CONFIG_FLOAT(init_x, "init_x");
CONFIG_FLOAT(init_y, "init_y");
CONFIG_FLOAT(init_r, "init_r");
CONFIG_FLOAT(init_loc_stddev, "init_loc_stddev");
CONFIG_FLOAT(init_r_stddev, "init_r_stddev");
CONFIG_FLOAT(d_short, "d_short");
CONFIG_FLOAT(d_long, "d_long");
CONFIG_FLOAT(ol_sigma, "ol_sigma");
CONFIG_FLOAT(gamma, "gamma");

// ---------START HELPER FUNCTIONS----------

void ParticleFilter::PrintParticles(){
  ROS_INFO("----------------------");
  for (int i = 0; i < int(particles_.size()); i++){
    ROS_INFO("[%d]: loc = (%f, %f), angle = %f, weight = %f", i, particles_[i].loc.x(), particles_[i].loc.y(), particles_[i].angle, particles_[i].weight);
  }
}

void ParticleFilter::UpdateOdometry(const Vector2f& odom_loc,
                                    const float odom_angle){
  if (!odom_initialized_) {
    curr_odom_angle_ = odom_angle;
    curr_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }

  curr_odom_loc_ = odom_loc;
  curr_odom_angle_ = odom_angle;
  curr_time_ = ros::Time::now().toSec();
  del_time_ = curr_time_ - prev_time_;
  odom_vel2f_ = GetOdomVel2f();
  odom_accel2f_ = GetOdomAccel2f();
  odom_vel_ = Vel2fToVel();
  odom_accel_ = Accel2fToAccel();
  del_odom_angle_ = curr_odom_angle_ - prev_odom_angle_;
  odom_omega_ = del_odom_angle_ / del_time_;

  ROS_INFO("----------------------------");
  ROS_INFO("prev_time_ = %f", prev_time_);
  ROS_INFO("curr_time_ = %f", curr_time_);
  ROS_INFO("del_time_ = %f", del_time_);

  ROS_INFO("prev_odom_loc_ = (%f, %f)", prev_odom_loc_.x(), prev_odom_loc_.y());
  ROS_INFO("curr_odom_loc_ = (%f, %f)", curr_odom_loc_.x(), curr_odom_loc_.y());

  ROS_INFO("prev_odom_vel2f_ = (%f, %f)", prev_odom_vel2f_.x(), prev_odom_vel2f_.y());
  ROS_INFO("odom_vel2f_ = (%f, %f)", odom_vel2f_.x(), odom_vel2f_.y());
  ROS_INFO("odom_vel_ = %f", odom_vel_);

  ROS_INFO("odom_accel2f_ = (%f, %f)", odom_accel2f_.x(), odom_accel2f_.y());
  ROS_INFO("odom_accel_ = %f", odom_accel_);

  ROS_INFO("prev_odom_angle_ = %f", prev_odom_angle_);
  ROS_INFO("curr_odom_angle_ = %f", curr_odom_angle_);
  ROS_INFO("del_odom_angle_ = %f", del_odom_angle_);
  ROS_INFO("odom_omega_ = %f", odom_omega_);

  prev_time_ = curr_time_;
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
  prev_odom_vel2f_ = odom_vel2f_;         
}

// ----------END HELPER FUNCTIONS-----------

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    curr_odom_loc_(0, 0),
    curr_odom_angle_(0),
    curr_time_(0),
    prev_time_(0),
    del_time_(0),
    prev_odom_vel2f_(0, 0),
    odom_vel2f_(0, 0),
    odom_accel2f_(0, 0),
    odom_vel_(0),
    odom_accel_(0),
    del_odom_angle_(0),
    odom_omega_(0) {}

void ParticleFilter::GetParticles(std::vector<Particle>* particles) const {
  *particles = particles_;
}


void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int32_t num_rays,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            std::vector<Vector2f>* scan_ptr,
                                            int32_t ray_interval) {
  std::vector<Vector2f>& scan = *scan_ptr;
  scan.resize((int)(num_rays / ray_interval));
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  const float max_ray_range = 10.0;
  const float angle_inc = (angle_max - angle_min) / (num_rays / ray_interval);

  float current_angle;
  Vector2f ray_start_point;
  Vector2f ray_end_point;
  line2f current_ray;
  Vector2f min_intersection_point;
  float min_intersection_dist = max_ray_range; 

  // iterate over each ray from laser
  for (int32_t i = 0; i < num_rays / ray_interval; i++) {
    current_angle = (float)i * angle_inc + angle_min + angle;

    // The laser vector starts at the laser scanner which is kLaserLoc offest from the
    // base link of the robot.
    ray_start_point = Vector2f(loc.x() + kLaserLoc.x() * cos(angle), 
                                        loc.y() +  kLaserLoc.x() * sin(angle));
    // The default end of the ray is the max length of the LIDAR.
    ray_end_point = ray_start_point + 
                    (Vector2f(cos(current_angle), sin(current_angle)) * max_ray_range);
    // Create the ray line2f object
    current_ray = line2f(ray_start_point.x(), ray_start_point.y(),
                      ray_end_point.x(), ray_end_point.y());

    min_intersection_dist = max_ray_range; 
    min_intersection_point = current_ray.p1;

    // iterate over each line in the map and return shortest distance to interection point
    int32_t num_map_lines = (int32_t)map_.lines.size();
    Vector2f intersection_point; 
    bool intersects;
    for (int32_t j = 0; j < num_map_lines; j++) {
      intersects = map_.lines[j].Intersection(current_ray, &intersection_point);

      if (intersects) {
        float intersection_dist = (intersection_point - ray_start_point).norm(); 
        if (intersection_dist <= min_intersection_dist && intersection_dist > 0.0){
          min_intersection_dist = intersection_dist;
          min_intersection_point = intersection_point; // world frame
        }
      }
    }
    scan[i] = min_intersection_point;
    //ROS_INFO("%d, %d, %ld", num_rays, int(i / ray_interval), scan.size());
  }
  
}

double Observe_Likelihood(double s_t_i, double pred_s_t_i, float s_min, float s_max, float d_short, float d_long, float sigma, float gamma){
  if ((s_t_i < s_min) || (s_t_i > s_max)) {
    // ROS_INFO("OL Branch: obs point not possible to see");
    return 0;
  }
  else if (s_t_i < pred_s_t_i - d_short){
    // ROS_INFO("OL Branch: obs pt much closer than pred pt");
    return pow(exp(-1 * pow(d_short, 2) / pow(sigma, 2)), gamma);
  }
  else if (s_t_i > pred_s_t_i + d_long){
    // ROS_INFO("OL Branch: obs pt much further than pred pt");
    return pow(exp(-1 * pow(d_long, 2) / pow(sigma, 2)), gamma);
  }
  else {
    // ROS_INFO("OL Branch: obs pt close than pred pt");
    return pow(exp(-1 * pow(s_t_i - pred_s_t_i, 2) / pow(sigma, 2)), gamma);
  }
}

void PrintBins(std::vector<WeightBin> bins) {
  ROS_INFO("----------------------");
  for (int i = 0; i < int(bins.size()); i++){
    ROS_INFO("[%d]: l = %f, u = %f", i, bins[i].lower_bound, bins[i].upper_bound);
  }
}

// update p_ptr->weight based on comparisons of predictedpointcloud with observedpointcloud
void ParticleFilter::Update(const std::vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr,
                            int32_t ray_interval) {
  

  std::vector<Vector2f> pred_scan;
  GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, (int32_t)ranges.size(), range_min,
                         range_max, angle_min, angle_max, &pred_scan, ray_interval);
 
  double p_s_t = 0.0;
  // for the given particle
  // for each predicted ray, compare to corresponding observed ray
  int32_t num_predicted_rays = (int32_t)pred_scan.size();
  for (int32_t i = 0; i < num_predicted_rays; i++) {
    double pred_s_t_i = (p_ptr->loc - pred_scan[i]).norm(); 
    double s_t_i = ranges[i * ray_interval];
    double ol = Observe_Likelihood(s_t_i, pred_s_t_i, range_min, range_max, CONFIG_d_short, CONFIG_d_long, CONFIG_ol_sigma, CONFIG_gamma);
    double log_ol;
    if (ol == 0.0) {
      continue;
    }
    log_ol = log(ol);
    //ROS_INFO(" [ray %d]: pred_s_t_i = %f, s_t_i = %f, ol = %f, log_ol = %f", i, pred_s_t_i, s_t_i, ol, log_ol);
    p_s_t += log_ol;
  }
  //ROS_INFO(" === p_s_t = %f", p_s_t);
  p_ptr->weight = exp(p_s_t);

}


void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // calculate total_sum of all weights in particles

  double start_width = 0.0;
  for (int32_t i = 0; i < num_particles_; i++) {
    weight_bins_[i].lower_bound = start_width;
    weight_bins_[i].upper_bound = weight_bins_[i].lower_bound + particles_[i].weight;
    start_width = weight_bins_[i].upper_bound;
  }

  //PrintBins(weight_bins_);
  
  // Resample with low variance
  float_t r = rng_.UniformRandom(0, (1.0 / num_particles_));
  float_t c = particles_[0].weight;
  int i = 0;

  for (int32_t m = 0; m < num_particles_; m++) {
    float_t u = r + (float_t)m * (1.0 / num_particles_);
    while (c < u) {
      i += 1;
      c += particles_[i].weight;
    }
    resampled_particles_[m] = particles_[i];
    resampled_particles_[m].reset_weight();
  }

  // Swap the newly sampled particles with the original (avoiding copy)
  particles_.swap(resampled_particles_);

}

void ParticleFilter::ObserveLaser(const std::vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {

  // Alternate method
  //ROS_INFO("====particles before update======");
  //PrintParticles();
  float min_move_dist = 0.0;
  double w_sum = 0.0;
  for (auto& particle : particles_) {
    if ((particle.prev_update_loc - particle.loc).norm() >= min_move_dist) {
      //ROS_INFO("Updating particle %f", particle.angle);
      Update(ranges, range_min, range_max, angle_min, angle_max, &particle, int32_t(10));
      particle.prev_update_loc = particle.loc;
      w_sum += particle.weight;
    }
  }
  
  //ROS_INFO("====particles after update======");
  //PrintParticles();
  

  if (w_sum != 0){
    for (auto& particle : particles_){
      particle.normalize_weight(w_sum);
    }
    //ROS_INFO("====particles after normalization======");
    //PrintParticles();
    if (sample_counter_ == sample_rate_) {
      Resample();
      sample_counter_ = 0;
    } else{
      sample_counter_ += 1;
    }
  } else{
    //ROS_INFO("====********w_max = 0 ??*******======");
  }
  
  
}

double ParticleFilter::GetMaxWeight() {
  double result = 0;
  for (auto& particle : particles_) {
    result += particle.weight;
  }
  return result;
}


void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {

  UpdateOdometry(odom_loc, odom_angle);
  for (auto& particle : particles_) {
    particle.model_movement(odom_vel2f_, odom_vel_, odom_omega_, prev_odom_angle_, 
                            del_time_, rng_, CONFIG_k1, CONFIG_k2, CONFIG_k3, CONFIG_k4);
  }
}


void ParticleFilter::Initialize(const std::string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);

  // Initialize the particles of size num_particles
  particles_ = std::vector<Particle>(FLAGS_num_particles);
  // Set the number of particles being used in the simulation.
  num_particles_ = (int32_t)FLAGS_num_particles;

  // Initialize the objects that are solely depedent on the num_particles.
  particles_ = std::vector<Particle>(num_particles_);
  resampled_particles_ = std::vector<Particle>(num_particles_);
  weight_bins_ = std::vector<WeightBin>(num_particles_);

  // Create randomly distributed particles around (init_x, init_y).
  for (int32_t i = 0; i < num_particles_; i++) {
    float_t x = rng_.Gaussian(loc.x(), CONFIG_init_loc_stddev);
    float_t y = rng_.Gaussian(loc.y(), CONFIG_init_loc_stddev);
    float_t r = rng_.Gaussian(angle, CONFIG_init_r_stddev);

    particles_[i].loc = Vector2f(x, y);
    particles_[i].angle = r;
    particles_[i].reset_weight();
    particles_[i].prev_update_loc = Vector2f(x, y);
  }
  
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:

  float sum_x, sum_y, sum_cos_angle, sum_sin_angle;
  sum_x = sum_y = sum_cos_angle = sum_sin_angle = 0;
  for (const auto& particle : particles_) {
    sum_x += particle.loc.x();
    sum_y += particle.loc.y();
    sum_cos_angle += cos(particle.angle);
    sum_sin_angle += sin(particle.angle);
  }

  loc = Vector2f(sum_x / num_particles_, sum_y / num_particles_);
  angle = atan2(sum_sin_angle / num_particles_, sum_cos_angle / num_particles_);
}


}  // namespace particle_filter
