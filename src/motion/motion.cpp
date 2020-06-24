
#include "motion.h"


Motion::Motion(const int32_t time_delay, const double max_linear_vel, const double max_accel, PIDParams & params): pid(params){
    _time_delay = time_delay;
    _max_accel = max_accel;
    _max_linear_vel = max_linear_vel;
}


void Motion::controller_eval(Eigen::VectorXd & current_pose, Eigen::VectorXd & goal, int64_t time_usec, Eigen::VectorXd & target_speed) {
    pid.loop(goal - current_pose, time_usec , target_speed);
}