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
\file    particle-filter.h
\brief   Particle Filter Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#ifndef __SRC_PARTICLE_FILTER_H__
#define __SRC_PARTICLE_FILTER_H__

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "shared/math/geometry.h"
#include "shared/util/random.h"
#include "vector_map/vector_map.h"
#include "particle.h"


namespace particle_filter {

struct WeightBin{
  double lower_bound;
  double upper_bound;
};


class ParticleFilter {
 public:
  // Default Constructor.
   ParticleFilter();

  /*
   * Get the new laser scan from the LIDAR and start the particle filter update and
   * resampling algorithm.
   *
   * @param ranges: the distances found along each LIDAR ray
   * @param range_min: the minimum range of the LIDAR
   * @param range_max: the maximum range of the LIDAR
   * @param angle_min: the starting angle of the LIDAR reading
   * @param angle_max: the ending angle of the LIDAR reading
   */
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Predict particle motion based on odometry.
  void Predict(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  /*
   * Initialize the particle filter based on the robots current location. Also
   * initialize all the structures dependent on the num_particles.
   *
   * @param map_file: the map file to load for simulation
   * @param loc: the location of the robot set in the GUI
   * @param angle: the angle of the robot's initial pose
   */
  void Initialize(const std::string& map_file,
                  const Eigen::Vector2f& loc,
                  const float angle);

  // Called to update variables based on odometry
  void UpdateOdometry(const Eigen::Vector2f& odom_loc,
                                      const float odom_angle);

  // Return the list of particles.
  void GetParticles(std::vector<Particle>* particles) const;

  // Get robot's current location.
  void GetLocation(Eigen::Vector2f* loc, float* angle) const;

  // Update particle weight based on laser.
  void Update(const std::vector<float>& ranges,
              float range_min,
              float range_max,
              float angle_min,
              float angle_max,
              Particle* p);

  // Resample particles.
  void Resample();

  // Print particles
  void PrintParticles();

  /*
   * Given the location and angle of a particle and a potential ray from the LIDAR, find
   * where this ray would intersect with the map.
   *
   * @param loc:
   * @param range_min:
   * @param range_max:
   * @param angle_min:
   * @param angle_max:
   * @param: obs_scan_ptr
   */
  void GetPredictedPointCloud(const Eigen::Vector2f& loc,
                              const float angle,
                              int32_t num_ranges,
                              float range_min,
                              float range_max,
                              float angle_min,
                              float angle_max,
                              std::vector<Eigen::Vector2f>* scan,
                              int32_t range_incr = 1);
  /*
   * Get the observed point cloud from the robots LIDAR.
   *
   * @param ranges:
   * @param range_min:
   * @param range_max:
   * @param angle_min:
   * @param angle_max:
   * @param: obs_scan_ptr
   */
  void GetObservedPointCloud(const Eigen::Vector2f& loc,
                            const float angle,
                            const std::vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            std::vector<Eigen::Vector2f>* obs_scan_ptr);
 private:

  // List of particles currently being tracked.
  std::vector<Particle> particles_;

  // Keep an extra structure for managing the resampled particles
  std::vector<Particle> resampled_particles_;

  // Bounds on the weight bins per particle
  std::vector<WeightBin> weight_bins_;

  // Map of the environment.
  vector_map::VectorMap map_;
  int32_t num_map_lines_;

  // Random number generator.
  util_random::Random rng_;

  // number of particles in the simulation. Supplied via command line.
  int32_t num_particles_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  Eigen::Vector2f curr_odom_loc_;
  float curr_odom_angle_;
  double curr_time_;
  double prev_time_;
  double del_time_;
  
  Eigen::Vector2f prev_odom_vel2f_;
  Eigen::Vector2f odom_vel2f_;
  Eigen::Vector2f odom_accel2f_;
  float odom_vel_;
  float odom_accel_;
  float del_odom_angle_;
  float odom_omega_;


  Eigen::Vector2f GetRayIntersection(const geometry::line2f& ray,
                                     float min_intersection_dist,
                                     const Eigen::Vector2f& laser_point);


};
}  // namespace slam

#endif   // __SRC_PARTICLE_FILTER_H__
