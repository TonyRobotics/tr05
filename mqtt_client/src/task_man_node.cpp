#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

#ifndef PI
#define PI 3.141592653
#endif

int main(int argc, char **argv){
    ros::init(argc, argv, "task_man_node");
    ros::NodeHandle nh;
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 5);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("move_base/PatrolPlanner/artificial_path", 5);
    geometry_msgs::Quaternion q;
    double theta = 0;
    double ax = 0; 
    double ay = 0;
    double az = theta*PI/180;
    q.x = sin(ax/2)*cos(ay/2)*cos(az/2) - cos(ax/2)*sin(ay/2)*sin(az/2);
    q.y = cos(ax/2)*sin(ay/2)*cos(az/2) + sin(ax/2)*cos(ay/2)*sin(az/2);
    q.z = cos(ax/2)*cos(ay/2)*sin(az/2) - sin(ax/2)*sin(ay/2)*cos(az/2);
    q.w = cos(ax/2)*cos(ay/2)*cos(az/2) + sin(ax/2)*sin(ay/2)*sin(az/2);
    // goal
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = 1;
    goal.pose.position.y = 1;
    goal.pose.orientation = q;
    // path
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    geometry_msgs::PoseStamped p1, p2, p3, p4, p5, p6, p7;
    p1.header.frame_id = "map";
    p1.pose.orientation = q;
    p1.pose.position.x = 0;
    p1.pose.position.y = 0;
    p2 = p1; p3 = p1; p4 = p1; p5 = p1; p6 = p1; p7 = p1;
    p2.pose.position.x = 1;
    p2.pose.position.y = 0;
    p3.pose.position.x = 2; 
    p3.pose.position.y = 0;
    p4.pose.position.x = 3;
    p4.pose.position.y = 0; 
    p5.pose.position.x = 3;
    p5.pose.position.y = -1; 
    p6.pose.position.x = 3;
    p6.pose.position.y = -2; 
    p7.pose.position.x = 3; 
    p7.pose.position.y = -3; 
    path.poses.push_back(p1);
    path.poses.push_back(p2);
    path.poses.push_back(p3);
    path.poses.push_back(p4);
    path.poses.push_back(p5);
    path.poses.push_back(p6);
    path.poses.push_back(p7);

    ros::Rate loop_rate(2);
    int count = 0;
    ROS_INFO("start pub");
    while(ros::ok()){
	if(count > 3){
	    break;
	}
	if(count < 2){
            ROS_INFO("pub path");
            path_pub.publish(path);
	}
	else{
            ROS_INFO("pub goal");
            goal_pub.publish(goal);
	}
        ros::spinOnce();
	loop_rate.sleep();
        ++count;
    }
    ROS_INFO("pub ok");
    return 0;
}
