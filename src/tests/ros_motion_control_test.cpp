#include "../motion/global_path_planer.h"
#include "../motion/motion.h"
#include <controllers/pid.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <memory>
#include <cstdlib>

Eigen::VectorXd current_pose = Eigen::VectorXd::Zero(3);
int64_t time_now = 0;
PIDParams pid_p(3);
Eigen::VectorXd goal = Eigen::VectorXd::Zero(3);
Eigen::VectorXd target_speed = Eigen::VectorXd::Zero(3);

Motion motion(1000, 0.2, 0.1, pid_p);

void controller_callback(const nav_msgs::Odometry & msg){
    current_pose << msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0;
    time_now = (ros::Time::now().sec + ros::Time::now().nsec*1e-9)*1e+6;
    motion.controller_eval(current_pose, goal, time_now, target_speed);
    ROS_INFO("Target speed: [%f, %f, %f]", target_speed(0), target_speed(1), target_speed(2));
}

int main(int argc , char *argv[]){

    ros::init(argc, argv, "nav_test2");
    ros::NodeHandle n;


    GraphMap graphmap(3);

    Eigen::VectorXd p0 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd p1 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd p2 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd p3 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd p4 = Eigen::VectorXd::Zero(3);
    p0 << 0, 0, 0;
    p1 << 1, -1, 0;
    p2 << 2, -1, 0;
    p3 << 3, -1, 0;
    p4 << 4, -0.5, 0;

    GraphPoint pg1(p1);
    GraphPoint pg2(p2);
    GraphPoint pg3(p3);
    GraphPoint pg4(p4);
    pg2.add_neighbor(pg1);
    pg3.add_neighbor(pg2);
    pg4.add_neighbor(pg2);

    int id1 = graphmap.append_point(pg1);
    int id2 = graphmap.append_point(pg2);
    int id3 = graphmap.append_point(pg3);
    int id4 = graphmap.append_point(pg4);


    std::vector<int> way = {id1, id2, id3, id4};

    GraphMapPtr graph_map_ptr = std::make_shared<GraphMap>(graphmap);

    GlobalPathPlaner planer(graph_map_ptr);
    std::cout << *(graph_map_ptr) << std::endl;
    planer.plan_by_way(way, p0);
    std::cout << planer.get_path_length() << std::endl;

    pid_p.kp << 1, 1, 0;
    pid_p.kd <<0.01,0.01,0;
    motion.pid.set_params(pid_p);

    ros::Publisher marker_pub = n.advertise<nav_msgs::Path>("path_visual", 1);
    ros::Subscriber sub = n.subscribe("/odom", 10, controller_callback);
    ros::Publisher control_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Rate r(500);
    double step_size = 0.002;
    double current_dist=0.0;

    while (ros::ok())
    {
        nav_msgs::Path msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "odom";

        std::vector<geometry_msgs::PoseStamped> poses((int)(planer.get_path_length()/step_size)+1);

        // Create the vertices for the points and lines
        for (double d = 0; d < planer.get_path_length(); d+=step_size) {
            Eigen::VectorXd point_coord;
            planer.get_point(d, point_coord);
            poses.at((int) (d / step_size)).pose.position.x = point_coord(0);
            poses.at((int) (d / step_size)).pose.position.y = point_coord(1);
            poses.at((int) (d / step_size)).header.stamp = ros::Time::now();
            poses.at((int) (d / step_size)).header.frame_id = "odom";
//            std::cout << point_coord <<"\n" <<std::endl;
        }

        Eigen::VectorXd target_coord;
        planer.get_point(current_dist, target_coord);
        goal(0) = target_coord(0);
        goal(1) = target_coord(1);

        msg.poses = poses;

        marker_pub.publish(msg);
        geometry_msgs::Twist msgt;

        current_dist+=step_size;
        if(current_dist >=planer.get_path_length()) {
            current_dist = planer.get_path_length();
        }


        msgt.linear.x = target_speed(0);
        msgt.linear.y = target_speed(1);
        msgt.angular.z = target_speed(2);

        ROS_INFO_STREAM("Target speed:"<<" linear_x ="<<target_speed(0)<<" linear_y ="<<target_speed(1)<<" angular="<<target_speed(2));
        control_pub.publish(msgt);


        r.sleep();
        ros::spinOnce();      //Notice this

    }


    return 0;
}
