#pragma once

#include <algorithms/pc_vec/pointcloud_vectorize.h>
#include <Eigen/Geometry>

class VecLaserSLAM {
public:
    VecLaserSLAM(double laser_scan_error);
    void estim_global_position(const Eigen::VectorXd & odom_local_position, Eigen::VectorXd & position);
    void update_map(const Eigen::VectorXd & odom_local_position, std::vector<Eigen::VectorXd> & polar_point_cloud);
    void features_points(std::vector<Eigen::VectorXd> & features);
    PointCloudToVec2d pc2v;
private:
    double _laser_scan_error;
    Eigen::VectorXd _estim_position;

};
