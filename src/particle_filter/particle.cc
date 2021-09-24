#include "particle.h"

    //Functions to change this function's location and angle

    //This function returns the change in x and y location, and internally updates the angle
    Particle::update_position(float_t speed, double del_time, float_t ang_vel) {
        return 0;
    }

    Eigen::Vector2f Particle::get_del_loc(float_t speed, double del_time, float_t ang_vel) {
        Eigen::Vector2f vel = Eigen::Vector2f(cos(angle) * speed, sin(angle) * speed);
        Eigen::Vector2f result = loc + (vel * del_time);
        result += trans_err_trans(speed, del_time);
        result += trans_err_rot(ang_vel, del_time);
        return result;
    }

    float_t Particle::get_del_angle(float_t speed, double del_time, float_t ang_vel) {
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

    Eigen::Vector2f Particle::trans_err_trans(float_t speed, double del_time) {
        double del_trans = speed * del_time;
        double result = gaussian(0, del_trans * k1);
        return Eigen::Vector2f(cos(angle), sin(angle)) * result;
    }

    Eigen::Vector2f Particle::trans_err_rot(float_t ang_vel, double del_time) {
        return 0; //EDIT THIS
    }

    float_t Particle::rot_err_rot(float_t std_dev) {
        return 0; //EDIT THIS
    }

    float_t Particle::rot_err_trans(float_t std_dev) {
        return 0; //EDIT THIS
    }