#include "particle.h"

#include "ros/ros.h"


void Particle::calc_weight() {
    weight = 3;
}

void Particle::reset_weight() {
    weight = 1;
}
void Particle::normalize_weight(double normalize_by) {
    weight /= normalize_by;
}

void Particle::model_movement(Eigen::Vector2f odom_velocity,
                                float_t speed,
                                float_t ang_vel,
                                float_t theta,
                                float_t del_time,
                                util_random::Random& rng,
                                float_t k1,
                                float_t k2,
                                float_t k3,
                                float_t k4) {
    // See https://www.google.com/url?q=https://docs.google.com/presentation/d/1Iruny27A_EAhpjE8mgiPvW0W9W0RKwKQRN2HEII-r-Y/edit?usp%3Dsharing&sa=D&source=editors&ust=1633377832336000&usg=AOvVaw0KNSsPeo7QVKp14Gls5_0P
    // slide 26-27
    Eigen::Vector2f base_link_translation = Eigen::Rotation2Df(-theta) * odom_velocity * del_time;                        
    float_t theta_magnitude = abs(theta);
    float_t translation_std = k1 * speed + k2 * theta_magnitude;

    float_t eps_x = rng.Gaussian(0, translation_std);
    float_t eps_y = rng.Gaussian(0, translation_std);
    float_t eps_theta = rng.Gaussian(0, k3 * speed + k4 * theta_magnitude);

    loc += Eigen::Rotation2Df(angle) * (base_link_translation + Eigen::Vector2f(eps_x, eps_y));
    angle += ang_vel * del_time + eps_theta;
}
