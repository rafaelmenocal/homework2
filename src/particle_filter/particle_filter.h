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
#ifndef __SRC_PARTICLE_FILTER_PARTICLE_FILTER_H__
#define __SRC_PARTICLE_FILTER_PARTICLE_FILTER_H__


#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "shared/math/line2d.h"
#include "shared/util/random.h"
#include "vector_map/vector_map.h"

#include "particle.h"


namespace particle_filter {


struct WeightBin {
  float_t lower_bound;
  float_t upper_bound;
};


class ParticleFilter {
  public:
    // Default Constructor.
    ParticleFilter();

    // Observe a new laser scan.
    void ObserveLaser(const std::vector<float>& ranges,
                      float range_min,
                      float range_max,
                      float angle_min,
                      float angle_max);

    // Predict particle motion based on odometry.
    void Predict(const Eigen::Vector2f& odom_loc,
                        const float odom_angle);

    // Initialize the robot location.
    void Initialize(const std::string& map_file,
                    const Eigen::Vector2f& loc,
                    const float angle);

    // Called to update variables based on odometry
    void UpdateOdometry(const Eigen::Vector2f& odom_loc,
                                        const float odom_angle);

    // Return the list of particles.
    void GetParticles(std::vector<Particle>* particles) const;

    /*
    * Loop over all the particles and get the average position and angle.
    * This is deemed the orientiation of the robot by the particles.
    * 
    * @param loc: the vector to fill with the average x and y location
    * @param angle: the average angle of all the particles.
    */
    void GetLocation(Eigen::Vector2f* loc, float* angle) const;

    // Update particle weight based on laser.
    void Update(const std::vector<float>& ranges,
                float range_min,
                float range_max,
                float angle_min,
                float angle_max,
                Particle* p,
                int32_t ray_interval);

    // Resample particles.
    void Resample();

    // Print particles
    void PrintParticles();

    // Get Max Weight of all particles
    double GetMaxWeight();

    // For debugging: get predicted point cloud from current location.
    void GetPredictedPointCloud(const Eigen::Vector2f& loc,
                                const float angle,
                                int32_t num_rays,
                                float range_min,
                                float range_max,
                                float angle_min,
                                float angle_max,
                                std::vector<Eigen::Vector2f>* scan,
                                int32_t ray_interval);

  private:

    // List of particles being tracked.
    std::vector<Particle> particles_;

    // Keep an extra structure for managing the resampled particles
    std::vector<Particle> resampled_particles_;

    // Bounds on the weight bins per particle
    std::vector<WeightBin> weight_bins_;

    // number of particles in the simulation. Supplied via command line.
    int32_t num_particles_;

    // Map of the environment.
    vector_map::VectorMap map_;

    // Random number generator.
    util_random::Random rng_;

    // Previous odometry-reported locations.
    Eigen::Vector2f prev_odom_loc_;
    float prev_odom_angle_;
    bool odom_initialized_;

    Eigen::Vector2f curr_odom_loc_;
    float curr_odom_angle_;
    double curr_time_;
    double prev_time_;
    double del_time_;
    
    Eigen::Vector2f del_odom_loc_;
    
    Eigen::Vector2f prev_odom_vel2f_;
    Eigen::Vector2f odom_vel2f_;
    Eigen::Vector2f odom_accel2f_;
    float odom_vel_;
    float odom_accel_;
    float del_odom_angle_;
    float odom_omega_;

    // A few small helper functions.
    inline float_t Vel2fToVel() const {
      return sqrt(pow(odom_vel2f_.x(), 2) + pow(odom_vel2f_.y(), 2));
    };

    inline float_t Accel2fToAccel() const {
      return sqrt(pow(odom_accel2f_.x(), 2) + pow(odom_accel2f_.y(), 2));
    }

    inline Eigen::Vector2f GetOdomVel2f() const {
      return (1.0 / del_time_) * 
        Eigen::Vector2f(curr_odom_loc_.x() - prev_odom_loc_.x(), curr_odom_loc_.y() - prev_odom_loc_.y());
    }

    inline Eigen::Vector2f GetOdomAccel2f() const {
      return (prev_odom_vel2f_ - odom_vel2f_) * del_time_;
  }

};
}  // namespace particle_filter

#endif // __SRC_PARTICLE_FILTER_PARTICLE_FILTER_H__
