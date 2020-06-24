#include "../slam/vectorized_laser_slam.h"
#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <memory>
#include <cstdlib>

std::vector<Eigen::VectorXd> point_polar_cloud;
Eigen::VectorXd current_pose_wheel_odom = Eigen::VectorXd::Zero(2);
Eigen::VectorXd current_pose_rf2o_odom = Eigen::VectorXd::Zero(2);
std::vector<Eigen::VectorXd> features;
int64_t time_now = 0;

void scan_callback(const sensor_msgs::LaserScan & msg);
void wheel_odom_callback(const nav_msgs::Odometry & msg);
void lidar_odom_callback(const nav_msgs::Odometry & msg);


int main(int argc , char *argv[]){

    ros::init(argc, argv, "nav_test_slam2");
    ros::NodeHandle n;

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_vectorized_map", 10);
    ros::Subscriber sub_rf2o = n.subscribe("/odom_rf2o", 10, lidar_odom_callback);
    ros::Subscriber sub_odom = n.subscribe("/odom", 10, wheel_odom_callback);
    ros::Subscriber sub_laser = n.subscribe("/scan_front", 10, scan_callback);
    ros::Rate r(10);

    VecLaserSLAM vec_slam(0.1);


    while (ros::ok())
    {
        visualization_msgs::Marker line_list;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "odom";

        line_list.header.frame_id = "/vec_map";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "vector_map_lines";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_strip.id = 0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.1;
        // Line list is blue
        line_list.color.b = 1.0;
        line_list.color.a = 1.0;

        for (long i = 0; i < point_polar_cloud.size(); ++i)
        {

            Eigne::VectorXd summary_odom_position = current_pose_rf2o_odom + current_pose_wheel_odom;
            vec_slam.update_map(summary_odom_position,point_polar_cloud);
//            Eigen::VectorXd global_pose = Eigen::VectorXd::Zero(2);
//            vec_slam.estim_global_position(summary_odom_position,global_pose);

            Eigen::VectorXd linep1;
            Eigen::VectorXd linep2;

            linep1 = vec_slam.pc2v.
            geometry_msgs::Point p;
            p.x = (int32_t)i - 50;
            p.y = y;
            p.z = z;

            points.points.push_back(p);
            line_strip.points.push_back(p);

            // The line list needs two points for each line
            line_list.points.push_back(p);
            p.z += 1.0;
            line_list.points.push_back(p);
        }

        vec_slam.features_points(features);

        for(long l=0; l<features.size(); l++){
            geometry_msgs::Point p1;
            p.x = features[l].init_point;
            p.y = features[l].finish_point;
            p.z = 0;
        }


        r.sleep();
        ros::spinOnce();      //Notice this

    }


    return 0;
}

void scan_callback(const sensor_msgs::LaserScan & msg){
    point_polar_cloud.clear();
    float range_min = msg.range_min;
    float range_max = msg.range_max;
    float angle_increment = msg.angle_increment;
    long count_points = std::abs(range_max-range_min)/angle_increment-1;
    long noninf_points=0;
    float* ranges = msg.ranges;
    for(long l=0; l<count_points; l++){
        float angle = l*angle_increment + range_min;
        if(ranges[l] >1e+6 || ranges[l]<-1e+6){
            Eigen::VectorXd point_polar;
            point_polar<<ranges[l], angle;
            point_polar_cloud.push_back();
            noninf_points++;
        }
    }
}

void wheel_odom_callback(const nav_msgs::Odometry & msg){
    current_pose_wheel_odom << msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0;
    time_now = (ros::Time::now().sec + ros::Time::now().nsec*1e-9)*1e+6;
}

void lidar_odom_callback(const nav_msgs::Odometry & msg){
    current_pose_rf2o_odom << msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0;
    time_now = (ros::Time::now().sec + ros::Time::now().nsec*1e-9)*1e+6;
}