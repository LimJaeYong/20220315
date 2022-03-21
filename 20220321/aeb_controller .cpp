#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"

geometry_msgs::Twist cmd_vel_msg;
std_msgs::Bool flag_AEB;
std_msgs::Float32 delta_range;
std_msgs::Float32 old_sonar_range;

void UltraSonarCallback(const sensor_msgs::Range::ConstPtr& msg) 
{
	ROS_INFO("Soner Seq: [%d]", msg->header.seq);
	ROS_INFO("Soner Range: [%f]", msg->range);	
	
	if(msg->range <= 1.0)
	{
		ROS_INFO("AEB_Activated");
		flag_AEB.data = true;
	}
	else {
		flag_AEB.data = false;
	}
	delta_range.data = old_sonar_range.data - msg->range;
	ROS_INFO("delta range : %f", delta_range.data);
	old_sonar_range.data = msg->range;
	
}

void CarControlCallback(const geometry_msgs::Twist& msg) 
{
	ROS_INFO("Cmd_vel :linear x [%f}", msg.linear.x);
	
	cmd_vel_msg = msg;
	ROS_INFO("Cmd_vel : linear x [%f]", cmd_vel_msg.linear.x);
}

int main(int argc, char **argv) {
	int count = 0;
	old_sonar_range.data = 0;
	
	ros::init(argc, argv, "aeb_controller");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	
	ros::Subscriber sub = n.subscribe("range", 1000, UltraSonarCallback);
	ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 10, &CarControlCallback);
	
	ros::Publisher pub_aeb_activation_flag = n.advertise<std_msgs::Bool>("/aeb_activation_flag", 1);
	ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel",10);
	ros::Publisher pub_delta_range = n.advertise<std_msgs::Float32>("/delta_range",1);
	
	while (ros::ok()) {
		if((count%10)==0)
		{
			pub_aeb_activation_flag.publish(flag_AEB);
		}
		
		if(flag_AEB.data == true)
		{
			cmd_vel_msg.linear.x = 0;
			pub_cmd_vel.publish(cmd_vel_msg);
		}
		else
		{
			pub_cmd_vel.publish(cmd_vel_msg);
		}
		
		
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}
	return 0;
}
