#include "global_path_planer.h"
#include <controllers/pid.h>

#pragma once

class Motion {
public:
    Motion(const int32_t time_delay, const double max_linear_vel, const double max_accel, PIDParams & params);
    void controller_eval(Eigen::VectorXd & current_pose, Eigen::VectorXd & goal, int64_t time_usec, Eigen::VectorXd & target_speed);
    PID pid;
private:
    int32_t _time_delay;
    double _max_linear_vel;
    double _max_accel;
};
