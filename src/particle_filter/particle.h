#include "eigen3/Eigen/Dense"

#include "shared/util/random.h"

class Particle {
    public:
    
        Eigen::Vector2f loc;
        float_t angle;
        double weight;

        //Functions to change this function's location and angle
        int update_position(float_t speed, double del_time, float_t ang_vel);
        Eigen::Vector2f get_del_loc(float_t speed, double del_time, float_t ang_vel);
        float_t get_del_angle(float_t speed, double del_time, float_t ang_vel);

        //Functions to edit the weight
        double calc_weight();
        double reset_weight();
        double normalize_weight(double normalize_by);

        //Functions to calculate the different types of error
        Eigen::Vector2f trans_err_trans(float_t speed, double del_time, util_random::Random rng, float_t k1);
        Eigen::Vector2f trans_err_rot(float_t ang_vel, double del_time);
        float_t rot_err_rot(float_t std_dev);
        float_t rot_err_trans(float_t std_dev);
};