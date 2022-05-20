#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <obstacle_detector/Obstacles.h>
#include <obstacle_detector/CircleObstacle.h>
#include <yamabiko/tf_position.h>

class person_tracking
{
public:
    person_tracking();

    void update();

private:
    //callback
    obstacle_detector::Obstacles obstacles;
    void obstacle_callback(const obstacle_detector::Obstacles &obstacle_message)
    {
        obstacles = obstacle_message;
    }

    //function
    geometry_msgs::Quaternion yaw2geometry_quat(double yaw);

    ros::NodeHandle nh;
    ros::Subscriber obstacle_sub;
    ros::Publisher pose_pub;

    double TRACK_RADIUS;
    int rate;
    std::string odom_id, base_link_id;

};

person_tracking::person_tracking()
{
    ros::NodeHandle pnh("~");

    pnh.param<double>("track_radius", TRACK_RADIUS, 1.0);
    pnh.param<int>("rate", rate, 20);
    pnh.param<std::string>("odom_frame_id", odom_id, "odom");
    pnh.param<std::string>("base_link_id", base_link_id, "base_link");

    obstacle_sub = nh.subscribe("tracked_obstacles", 10, &person_tracking::obstacle_callback, this);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("person_tracking/pose_out", 10);
}

void person_tracking::update()
{
    static tf_position nowPosition(odom_id, base_link_id, rate);
    static ros::Rate loop_rate(rate);

    //get now position
    geometry_msgs::PoseStamped now_pose = nowPosition.getPoseStamped();

    //select closest obstacle
    obstacle_detector::CircleObstacle closest_obstacle;
    double min_distance = 999999;

    int obstacles_size = obstacles.circles.size();
    //nothing to obstacle
    if(obstacles_size == 0){
        std::cout<<"tracking object is not founded"<<std::endl;
        return;
    }
    for(int i=0; i<obstacles_size; ++i){
        obstacle_detector::CircleObstacle obstacle = obstacles.circles[i];  
        double distance = sqrt(std::pow(obstacle.center.x - now_pose.pose.position.x, 2) + std::pow(obstacle.center.y - now_pose.pose.position.y, 2));
        if(distance < min_distance){
            min_distance = distance;
            closest_obstacle = obstacle;
        }
    }

    double angle = -(M_PI - atan2(closest_obstacle.center.y - now_pose.pose.position.y, closest_obstacle.center.x - now_pose.pose.position.x));

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = odom_id;
    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position.x = closest_obstacle.center.x + TRACK_RADIUS*cos(angle);
    target_pose.pose.position.y = closest_obstacle.center.y + TRACK_RADIUS*sin(angle);
    target_pose.pose.orientation = yaw2geometry_quat(angle);

    std::cout <<"tracked point x : "<< target_pose.pose.position.x <<", y : "<< target_pose.pose.position.y <<", yaw : "<< angle <<std::endl;
    
    pose_pub.publish(target_pose);

    loop_rate.sleep();

}

geometry_msgs::Quaternion person_tracking::yaw2geometry_quat(double yaw)
{
    double roll = 0;
    double pitch = 0;
    tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "person_tracking");

    person_tracking pt;
    while(ros::ok())
    {
        pt.update();
        ros::spinOnce();
    }

    return 0;
}