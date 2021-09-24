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

// ---------START HELPER FUNCTIONS----------

void PrintParticles(std::vector<Particle> particles){
  ROS_INFO("----------------------");
  for (int i = 0; i < int(particles.size()); i++){
    ROS_INFO("[%d]: loc = (%f, %f), angle = %f, weight = %f", i, particles[i].loc.x(), particles[i].loc.y(), particles[i].angle, particles[i].weight);
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
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    scan[i] = Vector2f(0, 0);
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  for (size_t i = 0; i < map_.lines.size(); ++i) {
    const line2f map_line = map_.lines[i];
    // The line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
    line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
    // Access the end points using `.p0` and `.p1` members:
    // ROS_INFO("P0: (%f, %f), P1: (%f, %f)", my_line.p0.x(), my_line.p0.y(), my_line.p1.x(), my_line.p1.y());
    // Check for intersections:
    bool intersects = map_line.Intersects(my_line);
    // You can also simultaneously check for intersection, and return the point
    // of intersection:
    Vector2f intersection_point; // Return variable
    intersects = map_line.Intersection(my_line, &intersection_point);
    if (intersects) {
      // ROS_INFO("Intersects at (%f, %f)", intersection_point.x(), intersection_point.y());
    } else {
      // ROS_INFO("No intersection");
    }
  }
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

  // here we will use obersvation likelihood model to update weights
  // for each ray from laser:
  //      determine s_t_i for p_ptr based on distance from car to 
  //            predicted intersection with wall
  //      calculate p(s_t_i|x_t) {where x_t == next_particles_?} by 
  //            proportional_to {ignore s_t_i < s_min ||J s_t_i > s_max
  //                             exp(.),
  //                             exp(.),
  //                             exp(.)}
  // find p(s_t|x) "probability of observing laser points given predicted location"
  //        using product of all p(s_t_i | x_t)s "inidividual probabilities of each ray"
  // update weight of p_ptr to be p_ptr.weight * p(s_t|x);

}

void ParticleFilter::Resample() {
  // create new_particles;
  // Resample the particles, proportional to their weights.
  // caculate total_sum of all weights in particles
  // create bins with width  = weight / total_sum
  // for each particle in particles (i = 1 to num_particles):
  //      randomly generate number 0 - 1;
  //      determine which bin that falls into -> bin_i
  //      new_particles.push_back(particles_[bin_i])
  // particles = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  float x = rng_.UniformRandom(0, 1);
  printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
         x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  
  // Call the Update and Resample steps as necessary.
  // for each particle p in partcles_ -> Update(ranges, range_min, range_max, angle_min, angle_max, p);
  // Resample();
}


void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  UpdateOdometry(odom_loc, odom_angle);

  // here we will use motion model to predict location of particle at next time step
  // start with n particles_ with position (px,py)
  // split particles into 4 groups for each error term (tet, ter, ret, rer)
  //     or apply each type of error for all particles
  // how do we represent/calculate tet, ter, ret, rer 
  //     (test with all errors = 0, or individually each error type)
  // predict location loc_hat and angle_hat of particles at next time step given odom velocity (given odom_location/angle) + error noise
  //     and store in new data structure?  next_particles_
  // return how likely it is for each particle to be at the next location loc_hat, angle_hat
  //     based on 1) starting location, 2) predicted location, 3) odometry

  for (auto& particle : particles_){
    // float trans_err_trans = 0.0;
    // float trans_err_rot = 0.0;
    // float rot_err_trans = 0.0;
    // float rot_err_rot = 0.0;
    particle.loc.x() += cos( particle.angle) * (odom_vel_ * del_time_);
    particle.loc.y() += sin( particle.angle) * (odom_vel_ * del_time_);
    //particles_[i].angle = r;
    particle.weight = 1.0;
  }

  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  // float k1 = CONFIG_k1;
  // float k2 = CONFIG_k2;
  // float k3 = CONFIG_k3;
  // float k4 = CONFIG_k4;

  // PrintParticles(particles_);

  // float x = rng_.Gaussian(0.0, 2.0);
  // printf("Random number drawn from Gaussian distribution with 0 mean and "
  //        "standard deviation of 2 : %f\n", x);
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
  }
  
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  loc = Vector2f(0, 0);
  angle = 0;
}


}  // namespace particle_filter
