#include "eigen3/Eigen/Dense"

class Particle {
    public:
    
    Eigen::Vector2f loc;
    float_t angle;
    double weight;

    //Functions to change this function's location and angle
    Eigen::Vector2f get_del_loc(float_t speed, double del_time);
    float_t get_del_angle(float_t ang_vel, double del_time);

    //Functions to edit the weight
    double calc_weight();
    double reset_weight();
    double normalize_weight(double normalize_by);

    private:

    //Functions to calculate the different types of error
    float_t trans_err_trans(float_t std_dev);
    float_t trans_err_rot(float_t std_dev);
    float_t rot_err_rot(float_t std_dev);
    float_t rot_err_trans(float_t std_dev);
};