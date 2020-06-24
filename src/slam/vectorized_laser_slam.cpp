#include "vectorized_laser_slam.h"

VecLaserSLAM::VecLaserSLAM(double laser_scan_error)
: pc2v(laser_scan_error, 50, 3e+2){
    _laser_scan_error = laser_scan_error;
}

void VecLaserSLAM::estim_global_position(const Eigen::VectorXd & odom_local_position, Eigen::VectorXd & position) {
    position = _estim_position;
}

void VecLaserSLAM::update_map(const Eigen::VectorXd & odom_local_position, std::vector<Eigen::VectorXd> & polar_point_cloud){
    Eigen::MatrixXd transform = Eigen::Rotation2Dd(odom_local_position(2)).toRotationMatrix();
    for(long l=0; l<polar_point_cloud.size(); l++){
        Eigen::VectorXd laser_point = Eigen::VectorXd::Zero(2);
        laser_point(0) = polar_point_cloud[l](0) * cos(polar_point_cloud[l](1));
        laser_point(1) = polar_point_cloud[l](0) * sin(polar_point_cloud[l](1));
        Eigen::VectorXd global_laser_point = transform.inverse()*laser_point;
        global_laser_point(0) += odom_local_position(0);
        global_laser_point(1) += odom_local_position(1);
        pc2v.add_point(global_laser_point);
    }
}

void VecLaserSLAM::features_points(std::vector<Eigen::VectorXd> & features){
    pc2v.intersection_points(features);
}