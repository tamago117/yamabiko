#include <iostream>
#include <algorithm>
#include <math.h>
#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include "yamabiko/PID.hpp"
#include "yamabiko/tf_position.h"


class move_to_pose
{
    public:
        move_to_pose();
        void update();

    private:
        //callback
        geometry_msgs::PoseStamped target_poseStamp;
        void pose_callback(const geometry_msgs::PoseStamped& pose_message)
        {
            target_poseStamp = pose_message;
        };

        geometry_msgs::Twist move(const geometry_msgs::PoseStamped& current_pose);
        void path_estimate(const geometry_msgs::PoseStamped& current_pose);
        double quat2yaw(geometry_msgs::Quaternion orientation);
        geometry_msgs::Quaternion yaw2geometry_quat(double yaw);

        //arrange angle -pi ~ +pi
        double arrangeAngle(double angle)
        {
            while(angle>M_PI) angle -= 2*M_PI;
            while(angle<-M_PI) angle += 2*M_PI;

            return angle;
        };

        ros::NodeHandle nh;
        ros::Subscriber pose_sub;
        ros::Publisher cmd_pub, path_pub;

        PID distance_pid, toTarget_angle_pid, diff_angle_pid;

        double kp_distance, kp_toTarget_angle, kp_diff_angle;
        double MAX_V, MAX_W;
        double FINISH_RADIUS;
        int rate;
        int path_samples;

        std::string odom_id, base_link_id;

};

move_to_pose::move_to_pose()
{
    ros::NodeHandle pnh("~");

    pnh.param<double>("kp_distance", kp_distance, 1.0);
    pnh.param<double>("kp_toTarget_angle", kp_toTarget_angle, 1.0);
    pnh.param<double>("kp_diff_angle", kp_diff_angle, -1.0);
    pnh.param<double>("max_v", MAX_V, 1.0);
    pnh.param<double>("max_w", MAX_W, 1.0);
    pnh.param<double>("finish_radius", FINISH_RADIUS, 0.1);

    pnh.param<int>("rate", rate, 20);
    pnh.param<int>("path_samples", path_samples, 500);
    pnh.param<std::string>("odom_frame_id", odom_id, "odom");
    pnh.param<std::string>("base_link_id", base_link_id, "base_link");

    pose_sub = nh.subscribe("move_to_pose/pose_in", 10, &move_to_pose::pose_callback, this);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("move_to_pose/cmd_vel", 10);
    path_pub = nh.advertise<nav_msgs::Path>("move_to_pose/path", 10);

    distance_pid.set(kp_distance, 0.0, 0.0);
    toTarget_angle_pid.set(kp_toTarget_angle, 0.0, 0.0);
    diff_angle_pid.set(kp_diff_angle, 0.0, 0.0);

}

void move_to_pose::update()
{
    static tf_position nowPosition(odom_id, base_link_id, rate);
    static ros::Rate loop_rate(rate);

    geometry_msgs::Twist cmd_vel = move(nowPosition.getPoseStamped());
    path_estimate(nowPosition.getPoseStamped());

    cmd_pub.publish(cmd_vel);

    ros::spinOnce();
    loop_rate.sleep();
}

geometry_msgs::Twist move_to_pose::move(const geometry_msgs::PoseStamped& current_pose)
{
    geometry_msgs::Twist cmd_vel;

    double x_diff = target_poseStamp.pose.position.x - current_pose.pose.position.x;
    double y_diff = target_poseStamp.pose.position.y - current_pose.pose.position.y;

    double distance = sqrt(pow(x_diff, 2.0) + pow(y_diff, 2.0));
    double theta = atan2(y_diff, x_diff);

    double alpha = arrangeAngle(theta - quat2yaw(current_pose.pose.orientation));
    double beta = arrangeAngle(quat2yaw(target_poseStamp.pose.orientation) - quat2yaw(current_pose.pose.orientation) - alpha);

    double toTarget_yawVel = toTarget_angle_pid.update(alpha, 0, 1/(double)rate);
    double diff_angle_yawVel = diff_angle_pid.update(beta, 0, 1/(double)rate);

    //std::cout << toTarget_yawVel << diff_angle_yawVel << std::endl;

    cmd_vel.linear.x = distance_pid.update(distance, 0, 1/(double)rate);
    cmd_vel.angular.z = toTarget_yawVel + diff_angle_yawVel;

    if(alpha > M_PI/2 or alpha < -M_PI/2){
        cmd_vel.linear.x = -cmd_vel.linear.x;
    }

    //clamp
    cmd_vel.linear.x = std::clamp(cmd_vel.linear.x, -MAX_V, MAX_V);
    cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -MAX_W, MAX_W);

    //finish
    if(distance < FINISH_RADIUS){
        cmd_vel.angular.z = 0;
    }

    return cmd_vel;
}

void move_to_pose::path_estimate(const geometry_msgs::PoseStamped& current_pose)
{
    nav_msgs::Path path;
    path.header.frame_id = odom_id;
    path.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped pose_temp;
    pose_temp = current_pose;

    int count = 0;
    while(count<path_samples)
    {
        pose_temp.header.stamp = ros::Time::now();

        geometry_msgs::Twist cmd_vel = move(pose_temp);
        double theta = quat2yaw(pose_temp.pose.orientation) + cmd_vel.angular.z*(1/(double)rate);
        pose_temp.pose.orientation = yaw2geometry_quat(theta);
        pose_temp.pose.position.x += cmd_vel.linear.x * cos(theta)*(1/(double)rate);
        pose_temp.pose.position.y += cmd_vel.linear.x * sin(theta)*(1/(double)rate);

        path.poses.push_back(pose_temp);

        double distance = sqrt(pow(target_poseStamp.pose.position.x - pose_temp.pose.position.x, 2)
                        + pow(target_poseStamp.pose.position.y - pose_temp.pose.position.y, 2));

        if(distance < FINISH_RADIUS) break;
        count++;

    }

    path_pub.publish(path);
}

double move_to_pose::quat2yaw(geometry_msgs::Quaternion orientation)
{
    double roll, pitch, yaw;
    tf::Quaternion quat;
    quaternionMsgToTF(orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference

    return yaw;
}

geometry_msgs::Quaternion move_to_pose::yaw2geometry_quat(double yaw)
{
    double roll = 0;
    double pitch = 0;
    tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_to_pose");

    move_to_pose mp;
    while(ros::ok())
    {
        mp.update();
    }

    return 0;
}
