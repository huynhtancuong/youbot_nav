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
#include <cmath>
#include <memory>
#include <cstdlib>

int main(int argc , char *argv[]){

    ros::init(argc, argv, "nav_test1");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<nav_msgs::Path>("path_visual", 1);
    ros::Rate r(30);

    GraphMap graphmap(3);

    Eigen::VectorXd p0 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd p1 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd p2 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd p3 = Eigen::VectorXd::Zero(3);
    p0 << 0, 0, 0;
    p1 << 0, 1, 0;
    p2 << 1, 2, 0;
    p3 << 3, 4, 0;

    GraphPoint pg1(p1);
    GraphPoint pg2(p2);
    GraphPoint pg3(p3);
    pg3.add_neighbor(pg1);
    pg3.add_neighbor(pg2);

    int id1 = graphmap.append_point(pg1);
    int id2 = graphmap.append_point(pg2);
    int id3 = graphmap.append_point(pg3);

    std::vector<int> way = {id1, id3, id2};

    GraphMapPtr graph_map_ptr = std::make_shared<GraphMap>(graphmap);

    GlobalPathPlaner planer(graph_map_ptr);
    std::cout << *(graph_map_ptr) << std::endl;
    planer.plan_by_way(way, p0);
    std::cout << planer.get_path_length() << std::endl;

    PIDParams pid_p(3);
    pid_p.kp << 1,1,0;
    Motion motion(1000, 0.2, 0.1, pid_p);


    while (ros::ok())
    {
        nav_msgs::Path msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "odom";

        double step_size = 0.05;
        std::vector<geometry_msgs::PoseStamped> poses((int)(planer.get_path_length()/step_size)+1);


        for (double d = 0; d < planer.get_path_length(); d+=step_size)
        {
            Eigen::VectorXd point_coord;
            planer.get_point(d, point_coord);

            poses.at((int)(d/step_size)).pose.position.x = point_coord(0);
            poses.at((int)(d/step_size)).pose.position.y = point_coord(1);
            poses.at((int)(d/step_size)).header.stamp = ros::Time::now();
            poses.at((int)(d/step_size)).header.frame_id = "odom";
//            std::cout << point_coord <<"\n" <<std::endl;
        }

        msg.poses = poses;

        marker_pub.publish(msg);

        r.sleep();
    }


    return 0;
}
