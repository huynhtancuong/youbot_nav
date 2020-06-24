#include <controllers/pid.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <cmath>
#include <memory>
#include <cstdlib>

Eigen::VectorXd current_pose = Eigen::VectorXd::Zero(3);
int64_t time_now=0;
PIDParams pid_p(3);
Eigen::VectorXd goal = Eigen::VectorXd::Zero(3);
Eigen::VectorXd target_speed = Eigen::VectorXd::Zero(3);
PID pid(pid_p);

void controller_callback(const nav_msgs::Odometry & msg){
    current_pose << msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0;
    time_now = (ros::Time::now().sec + ros::Time::now().nsec*1e-9)*1e+6;
    pid.loop(goal - current_pose, time_now , target_speed);
    ROS_INFO("Current pose: [%f, %f, %f]", target_speed(0), target_speed(1), target_speed(2));
}

int main(int argc , char *argv[]) {

    ros::init(argc, argv, "controller_pid_test1");
    ros::NodeHandle n;

    pid_p.kp << 1, 1, 0;
    pid_p.ki <<0.01,0.01,0;
    pid.set_params(pid_p);
    goal<<1,0,0;

    ros::Subscriber sub = n.subscribe("/odom", 10, controller_callback);
    ros::Publisher control_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate r(1000);
    while(ros::ok()){
        geometry_msgs::Twist msg;
        msg.linear.x = target_speed(0);
        msg.linear.y = target_speed(1);
        msg.angular.z = target_speed(2);

        ROS_INFO_STREAM("Target speed:"<<" linear_x ="<<target_speed(0)<<" linear_y ="<<target_speed(1)<<" angular="<<target_speed(2));
        control_pub.publish(msg);

        r.sleep();
        ros::spinOnce();      //Notice this
    }
}
