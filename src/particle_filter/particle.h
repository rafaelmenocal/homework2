
#ifndef __SRC_PARTICLE_FILTER_PARTICLE_H__
#define __SRC_PARTICLE_FILTER_PARTICLE_H__

#include "eigen3/Eigen/Dense"

#include "shared/util/random.h"

class Particle {
    public:
    
        Eigen::Vector2f loc;
        float_t angle;
        double weight = 1.0;
        Eigen::Vector2f prev_update_loc;

        //Functions to change this function's location and angle
        int update_position(float_t speed, double del_time, float_t ang_vel);
        Eigen::Vector2f get_del_loc(float_t speed, double del_time, float_t ang_vel);
        float_t get_del_angle(float_t speed, double del_time, float_t ang_vel);

        //Functions to edit the weight
        void calc_weight();
        void reset_weight();
        void normalize_weight(double normalize_by);

        void model_movement(Eigen::Vector2f odom_velocity,
                            float_t speed,
                            float_t ang_vel,
                            float_t theta,
                            float_t del_time,
                            util_random::Random& rng,
                            float_t k1,
                            float_t k2,
                            float_t k3,
                            float_t k4);
        
        void model_movement(Eigen::Vector2f odom_velocity,
                            float_t speed,
                            float_t ang_vel,
                            float_t theta,
                            float_t del_time,
                            util_random::Random& rng,
                            Eigen::Vector2f del_odom_loc,
                            float_t del_odom_angle,
                            float_t k1,
                            float_t k2,
                            float_t k3,
                            float_t k4);

};

#endif // __SRC_PARTICLE_FILTER_PARTICLE_H__
