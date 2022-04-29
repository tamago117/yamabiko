#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "math.h"



class line_tracker
{
    public:
        line_tracker();
        void update();
    private:
        nav_msgs::Odometry pos;
        void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
        {
            pos.header = msg->header;
            pos.child_frame_id = msg->child_frame_id;
            pos.pose = msg->pose;
            pos.twist = msg->twist;
        };

        int near_position(double goal_x, double goal_y)
        {
            double difx = pos.pose.pose.position.x - goal_x;
            double dify = pos.pose.pose.position.y - goal_y;

            return (sqrt(difx * difx + dify * dify) < 0.3);
        };

        void line_GL(double x, double y, double th);

        ros::NodeHandle nh;
	    ros::Subscriber odom_sub;
	    ros::Publisher cmd_pub;

        geometry_msgs::Twist cmd_vel;

        //parameter
        // 直線を指定するための値
	    double target_x;
	    double target_y;
	    double target_theta;

        // 制御のパラメータ(調整必須)
        double k_eta;
        double k_phai;
        double k_w;

        // 速度と角速度の最大値
        double v_max;
        double w_max;

};

line_tracker::line_tracker()
{
    ros::NodeHandle pnh("~");

    pnh.param<double>("target_x", target_x, 1.0);
    pnh.param<double>("target_y", target_y, 1.0);
    pnh.param<double>("target_theta", target_theta, 0.5);
    pnh.param<double>("k_eta", k_eta, 0.8);
    pnh.param<double>("k_phai", k_phai, 0.3);
    pnh.param<double>("k_w", k_w, 0.2);
    pnh.param<double>("v_max", v_max, 0.5);
    pnh.param<double>("w_max", w_max, 1.0);

    odom_sub = nh.subscribe("line_tracker/odom_in", 1000, &line_tracker::odom_callback, this);
	cmd_pub = nh.advertise<geometry_msgs::Twist>("line_tracker/cmd_vel", 1000);
}

void line_tracker::update()
{
    static ros::Rate loop_rate(100);

	while (ros::ok())
	{

		line_GL(target_x, target_y, target_theta);
		cmd_pub.publish(cmd_vel);

        ros::spinOnce();
		loop_rate.sleep();
	}
}

//　直線追従を行う関数
void line_tracker::line_GL(double x, double y, double th)
{
	tf::Quaternion quat(pos.pose.pose.orientation.x, pos.pose.pose.orientation.y, pos.pose.pose.orientation.z, pos.pose.pose.orientation.w);
	tf::Matrix3x3 m(quat);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	// 現在のロボットの位置、姿勢
	double x0 = pos.pose.pose.position.x;
	double y0 = pos.pose.pose.position.y;
	double theta = yaw;

	// 速度
	double v0 = v_max;

	// 現在の角速度
	double w0 = pos.twist.twist.angular.z;

	// ロボットと直線の距離
	double eta = 0;
	if (th == M_PI / 2.0)
		eta = x0 - x;
	else
		eta = (-tan(th) * x0 + y0 - y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1);
	if (eta > 4.0)
		eta = 4.0;
	else if (eta < -4.0)
		eta = -4.0;

	// 直線に対するロボットの向き
	double phai = theta - th;
	while (phai <= -M_PI || M_PI <= phai)
	{
		if (phai <= -M_PI)
			phai = phai + 2 * M_PI;
		else
			phai = phai - 2 * M_PI;
	}

	// 目標となるロボットの角速度と現在の角速度の差
	double w_diff = w0;

	// 角速度
	double w = w0 + (-k_eta * eta - k_phai * phai - k_w * w_diff) * 0.01;
	if (w > w_max)
		w = w_max;
	else if (w < -w_max)
		w = -w_max;

	// 並進速度
	double v = v0 - abs(w0);
	if (v > v_max)
		v = v_max;
	else if (v < -v_max)
		v = 0.0;
		
	std::cout << "eta: " << eta << "  phai; " << phai << "  w_diff:" << w_diff << std::endl;
	std::cout << "v: " << v << "   w: " << w << std::endl;
	std::cout << "(x,y,theta) = (" << x0 << "," << y0 << "," << theta << ")" << std::endl;
	std::cout << "------------------------------" << std::endl;

	// 送信する値
	cmd_vel.linear.x = v;
	cmd_vel.angular.z = w;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_tracker");
    line_tracker lt;

	lt.update();

	return 0;
}