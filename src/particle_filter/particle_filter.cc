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
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "ros/ros.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"
// #include "particle.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");
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

float Vel2f_To_Vel(const Vector2f& velocity) {
  return sqrt(pow(velocity.x(), 2) + pow(velocity.y(), 2));
}

float Accel2f_To_Accel(const Vector2f& accel) {
  return sqrt(pow(accel.x(), 2) + pow(accel.y(), 2));
}

Vector2f GetOdomVel2f(const Vector2f& last_loc, const Vector2f& current_loc, float del_time) {
  return (1/ del_time) * Vector2f(current_loc.x() - last_loc.x(), current_loc.y() - last_loc.y());
}

Vector2f GetOdomAccel2f(const Vector2f& last_vel, const Vector2f& current_vel, float del_time) {
  return (last_vel - current_vel) * del_time;
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
  odom_vel2f_ = GetOdomVel2f(prev_odom_loc_, curr_odom_loc_, del_time_);
  odom_accel2f_ = GetOdomAccel2f(prev_odom_vel2f_, odom_vel2f_, del_time_);
  odom_vel_ = Vel2f_To_Vel(odom_vel2f_);
  odom_accel_ = Accel2f_To_Accel(odom_accel2f_);
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

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}


void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            float num_ranges,
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

  const float angle_inc = (angle_max - angle_min) / num_ranges;
  scan.resize(int(num_ranges));

  float max_ray_range = 10.0;

  //iterate over each ray from laser
  for (int i = 0; i < int(num_ranges); ++i) {
    float current_angle = i * angle_inc + angle_min + angle;
    Vector2f laser_point = Vector2f(loc.x() + kLaserLoc.x(), loc.y() + kLaserLoc.y());
    // current_ray is defined using odometry x and y
    Vector2f ray_end_point = Vector2f(loc.x() + max_ray_range * cos(current_angle), loc.y() + max_ray_range * sin(current_angle));
    line2f current_ray(laser_point.x(), laser_point.y(), ray_end_point.x(), ray_end_point.y()); 
    float min_intersection_dist = max_ray_range; 
    Vector2f min_intersection_point = current_ray.p1;

    // iterate over each line in the map and return shortest distance to interection point
    for (size_t i = 0; i < map_.lines.size(); ++i) {
      const line2f map_line = map_.lines[i];
      Vector2f intersection_point; 

      // redefine current_ray in terms of map coordinates??
      bool intersects = map_line.Intersection(current_ray, &intersection_point);
      if (intersects) {
        // ROS_INFO("Intersects at (%f, %f)", intersection_point.x(), intersection_point.y());
        float intersection_dist = (intersection_point - laser_point).norm(); 
        if (intersection_dist <= min_intersection_dist){
          min_intersection_dist = intersection_dist;
          // min_intersection_point = intersection_point - laser_point; // from laser
          min_intersection_point = intersection_point; // world frame
        }
      } else {
        // ROS_INFO("No intersection");
      }
    }

    scan[i] = min_intersection_point; 
  }
  
}

void ParticleFilter::GetObservedPointCloud(const Vector2f& loc,
                                           const float angle,
                                           const vector<float>& ranges,
                                           float num_ranges,
                                           float range_min,
                                           float range_max,
                                           float angle_min,
                                           float angle_max,
                                           vector<Vector2f>* obs_scan_ptr) {
  
  vector<Vector2f>& obs_scan = *obs_scan_ptr;
  obs_scan.resize(int(num_ranges));
  Vector2f laser_point = Vector2f(loc.x() + kLaserLoc.x(), loc.y() + kLaserLoc.y());

  const float index_inc = (ranges.size() / (num_ranges - 1));
  const float angle_inc = (angle_max - angle_min) / (num_ranges - 1);
  // ROS_INFO("ranges.size() = %ld", ranges.size());
  // ROS_INFO("num_ranges = %f", num_ranges);
  // ROS_INFO("angle_min = %f", angle_min);
  // ROS_INFO("angle_max = %f", angle_max);
  // ROS_INFO("index_inc = %f", index_inc);
  // ROS_INFO("angle_inc = %f", angle_inc);
  for (int i = 0; i < int(num_ranges); i++){
    // ROS_INFO("(i * index_inc) = %d", int(i * index_inc));
    // ROS_INFO("angle = i * angle_inc + angle_min = %f", i * angle_inc + angle_min);
    float current_angle = i * angle_inc + angle_min;
    obs_scan[i] = laser_point + Vector2f(ranges[int(i * index_inc)] * cos(current_angle), ranges[int(i * index_inc)] * sin(current_angle));
  }


}

double Observe_Likelihood(double s_t_i, double pred_s_t_i, float s_min, float s_max, float d_short, float d_long, float sigma, float gamma){
  if ((s_t_i < s_min) || (s_t_i > s_max)) {
    ROS_INFO("OL Branch: obs point not possible to see");
    return 0;
  }
  else if (s_t_i < pred_s_t_i - d_short){
    ROS_INFO("OL Branch: obs pt much closer than pred pt");
    return pow(exp(-1 * pow(d_short, 2) / pow(sigma, 2)), gamma);
  }
  else if (s_t_i > pred_s_t_i + d_long){
    ROS_INFO("OL Branch: obs pt much further than pred pt");
    return pow(exp(-1 * pow(d_long, 2) / pow(sigma, 2)), gamma);
  }
  else {
    ROS_INFO("OL Branch: obs pt close than pred pt");
    return pow(exp(-1 * pow(s_t_i - pred_s_t_i, 2) / pow(sigma, 2)), gamma);
  }
}

struct bin{
  double lower_bound;
  double upper_bound;
};

void PrintBins(std::vector<bin> bins){
  ROS_INFO("----------------------");
  for (int i = 0; i < int(bins.size()); i++){
    ROS_INFO("[%d]: l = %f, u = %f", i, bins[i].lower_bound, bins[i].upper_bound);
  }
}

// update p_ptr->weight based on comparisons of predictedpointcloud with observedpointcloud
void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  
  vector<Vector2f> pred_scan;
  float num_ranges = ranges.size() / 10.0;  // some of the ranges
  GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, num_ranges, range_min, range_max, angle_min, angle_max, &pred_scan);

  vector<Vector2f> observed_scan;
  GetObservedPointCloud(p_ptr->loc, p_ptr->angle, ranges, num_ranges, range_min, range_max, angle_min, angle_max, &observed_scan);

  double p_s_t = 0.0;
  // for the given particle
  // for each predicted ray, compare to corresponding observed ray
  for (int i = 0; i < int(num_ranges); ++i) {
    double pred_s_t_i = ((p_ptr->loc + kLaserLoc) - pred_scan[i]).norm(); 
    double s_t_i = ((p_ptr->loc + kLaserLoc) - observed_scan[i]).norm();
    double ol = Observe_Likelihood(s_t_i, pred_s_t_i, range_min, range_max, CONFIG_d_short, CONFIG_d_long, CONFIG_ol_sigma, CONFIG_gamma);
    double log_ol;
    if (ol == 0.0) {
      continue;
    }
    log_ol = log(ol);
    ROS_INFO(" [ray %d]: pred_s_t_i = %f, s_t_i = %f, ol = %f, log_ol = %f", i, pred_s_t_i, s_t_i, ol, log_ol);
    p_s_t += log_ol;
  }
  ROS_INFO(" === p_s_t = %f", p_s_t);
  ROS_INFO(" === weight = %f", exp(p_s_t));
  p_ptr->weight = exp(p_s_t);

}


void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // caculate total_sum of all weights in particles

  // create bins data structure vector<bin>
  std::vector<bin> bins(particles_.size());

  double start_width = 0.0;
  int p_i = 0;
  // update bins with particle weights as min_max
  for (auto& bin : bins){
    bin.lower_bound = start_width;
    bin.upper_bound = bin.lower_bound + particles_[p_i].weight;
    p_i++;
    start_width = bin.upper_bound;
  }

  PrintBins(bins);

  std::vector<Particle> particles_copy_;
  
  // // resample without low variance
  // for (int i = 0; i < int(particles_.size()); i++){
  //   // pick a random number between 0 - total width
  //   float x = rng_.UniformRandom(0, (bins.end()--)->upper_bound);

  //   // get the index of bins which x falls into
  //   for (int j = 0; j < int(bins.size()); j++) {
  //     if (bins[j].lower_bound <= x && x <= bins[j].upper_bound) {
  //       particles_copy_.push_back(particles_[j]);
  //       particles_copy_[i].weight = 1.0;
  //     }
  //   }
  // }

  // Resample with low variance
  double x = rng_.UniformRandom(0, ((bins.end()--)->upper_bound) / FLAGS_num_particles);
  for (int i = 0; i < int(particles_.size()); i++){
      for (int j = 0; j < int(bins.size()); j++) {
        double sample = x * (j + 1);
        if (bins[j].lower_bound <= sample && sample <= bins[j].upper_bound) {
          particles_copy_.push_back(particles_[j]);
          particles_copy_[i].weight = 1.0; // /num_particles; ?
        }
    }
  }

  particles_ = particles_copy_;

}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  
  // Initial method
  // for (auto& particle : particles_){
  //   if ((particle.prev_update_loc - particle.loc).norm() >= 0.15) {
  //     Update(ranges, range_min, range_max, angle_min, angle_max, &particle);
  //     particle.prev_update_loc = particle.loc;

  //     double w_max = GetMaxWeight();
  //     for (auto& particle : particles_){
  //       particle.normalize_weight(w_max);
  //     }
  //     PrintParticles();

  //     if (rng_.UniformRandom(0.0, 1.0) <= 1.0) {
  //       Resample();
  //     }

  //   }
  // }

  
  // Alternate method
  ROS_INFO("====particles before update======");
  PrintParticles();
  double w_max = 0.0;
  int i = 0;
  for (auto& particle : particles_){
    if ((particle.prev_update_loc - particle.loc).norm() >= 0.15) {
      ROS_INFO("Updating particle %d", i);
      Update(ranges, range_min, range_max, angle_min, angle_max, &particle);
      particle.prev_update_loc = particle.loc;
    }
    i++;
  }
  
  // ROS_INFO("====particles after update======");
  // PrintParticles();
  ROS_INFO("w_max = %f", w_max);
  w_max = GetMaxWeight();
  ROS_INFO("w_max = %f", w_max);

  // if (w_max != 0){
  //   for (auto& particle : particles_){
  //     particle.normalize_weight(w_max);
  //   }
  //   ROS_INFO("====particles after normalization======");
  //   PrintParticles();
  // }
  
  // if (rng_.UniformRandom(0.0, 1.0) <= 1.0) {
  //   Resample();
  // }

  // ROS_INFO("====particles after resample======");
  // PrintParticles();
  
}

double ParticleFilter::GetMaxWeight() {
  double result = 0;
  for (auto& particle : particles_) {
    result = particle.weight > result ? particle.weight : result;
  }
  return result;
}


void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {

  UpdateOdometry(odom_loc, odom_angle);

  // here we will use motion model to predict location of particle at next time step
  for (auto& particle : particles_){
    Eigen::Vector2f tet = particle.trans_err_trans(odom_vel_, del_time_, rng_, CONFIG_k1);
    Eigen::Vector2f ter = particle.trans_err_rot(odom_omega_, del_time_, rng_, CONFIG_k1);
    float_t rer = particle.rot_err_rot(odom_omega_, del_time_, rng_, CONFIG_k3);
    float_t ret = particle.rot_err_rot(odom_vel_, del_time_, rng_, CONFIG_k4);

    particle.loc += odom_vel_ * del_time_ * Vector2f(cos(particle.angle), sin(particle.angle)) + tet + ter;
    particle.angle += odom_omega_ * del_time_ + ret + rer;
  }

  // return how likely it is for each particle to be at the next location loc_hat, angle_hat
  //     based on 1) starting location, 2) predicted location, 3) odometry

}


void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);

  // Initialize the particles of size num_particles
  particles_ = std::vector<Particle>(FLAGS_num_particles);

  // Create randomly distributed particles around (init_x, init_y).
  for (int i = 0; i < int(particles_.size()); i++){
    float x = rng_.Gaussian(loc.x(), CONFIG_init_loc_stddev);
    float y = rng_.Gaussian(loc.y(), CONFIG_init_loc_stddev);
    float r = rng_.Gaussian(angle, CONFIG_init_r_stddev);

    particles_[i].loc = Vector2f(x, y);
    particles_[i].angle = r;
    particles_[i].weight = 1.0;
    particles_[i].prev_update_loc = Vector2f(x, y);
  }
  
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:

  int N = particles_.size();
  float sum_x = 0;
  float sum_y = 0;
  float sum_cos_angle = 0;
  float sum_sin_angle = 0;
  for (auto& particle : particles_){
    sum_x += particle.loc.x();
    sum_y += particle.loc.y();
    sum_cos_angle += cos(particle.angle);
    sum_sin_angle += sin(particle.angle);
  }

  loc = Vector2f(sum_x / N, sum_y / N);
  angle = atan2(sum_sin_angle / N, sum_cos_angle / N);
}


}  // namespace particle_filter
