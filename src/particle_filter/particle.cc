#include "particle.h"

    //Functions to change this function's location and angle
    Eigen::Vector2f Particle::get_del_loc(Eigen::Vector2f vel, double del_time) {
        float_t result = loc + (vel * del_time);
    }

    float_t Particle::get_del_angle(float_t ang_vel, double del_time) {
        //Right now, we aren't turning. EDIT THIS eventually.
        return 0;
    }

    double Particle::calc_weight() {
        weight = 3; //EDIT THIS
    }

    double Particle::reset_weight() {
        weight = 1; //EDIT THIS maybe, though if this is a good default weight, it's actually fine
    }
    double Particle::normalize_weight(double normalize_by) {
        weight /= normalize_by; //Now, how we get the value to normalize by, I'm not sure
    }

    float_t Particle::trans_err_trans(float_t std_dev) {
        return 0; //EDIT THIS
    }

    float_t Particle::trans_err_rot(float_t std_dev) {
        return 0; //EDIT THIS
    }

    float_t Particle::rot_err_rot(float_t std_dev) {
        return 0; //EDIT THIS
    }

    float_t Particle::rot_err_trans(float_t std_dev) {
        return 0; //EDIT THIS
    }