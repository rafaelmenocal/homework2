#include "particle.h"

    //Functions to change this function's location and angle

    //This function returns the change in x and y location, and internally updates the angle
    int Particle::update_position(float_t speed, double del_time, float_t ang_vel) {
        return 0;
    }

    // Eigen::Vector2f Particle::get_del_loc(float_t speed, double del_time, float_t ang_vel) {
    //     Eigen::Vector2f vel = Eigen::Vector2f(cos(angle) * speed, sin(angle) * speed);
    //     Eigen::Vector2f result = loc + (vel * del_time);
    //     result += trans_err_trans(speed, del_time);
    //     result += trans_err_rot(ang_vel, del_time);
    //     return result;
    // }

    float_t Particle::get_del_angle(float_t speed, double del_time, float_t ang_vel) {
        //Right now, we aren't turning. EDIT THIS eventually.
    //     float_t angle = Eigen::Vector2f(cos(angle) * speed, sin(angle) * speed);
    //     float_t result = loc + (vel * del_time);
    //     result += trans_err_trans(speed, del_time);
    //     result += trans_err_rot(ang_vel, del_time);
    //     return result;
        return 0;
    }

    double Particle::calc_weight() {
        weight = 3; //EDIT THIS
        return 0;
    }

    void Particle::reset_weight() {
        weight = 1.0; // / num_particles; ?
    }

    void Particle::normalize_weight(double normalize_by) {
        weight /= normalize_by;
    }

    Eigen::Vector2f Particle::trans_err_trans(
        float_t speed, double del_time, util_random::Random& rng, float_t k1) {
        double del_trans = speed * del_time;
        double result = rng.Gaussian(0, del_trans * k1);
        return Eigen::Vector2f(cos(angle), sin(angle)) * result;
    }

    Eigen::Vector2f Particle::trans_err_rot(
        float_t ang_vel, double del_time, util_random::Random& rng, float_t k2) {
        double del_rot = abs(ang_vel * del_time);
        double result = rng.Gaussian(0, del_rot * k2);
        return Eigen::Vector2f(cos(angle), sin(angle)) * result;
    }

    float_t Particle::rot_err_rot(float_t ang_vel, double del_time, util_random::Random& rng, float_t k3) {
        double del_rot = abs(ang_vel * del_time);
        double result = rng.Gaussian(0, del_rot * k3);
        return result;
    }

    float_t Particle::rot_err_trans(float_t speed, double del_time, util_random::Random& rng, float_t k4) {
        double del_trans = speed * del_time;
        double result = rng.Gaussian(0, del_trans * k4);
        return result;
    }