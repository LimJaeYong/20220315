#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"

std_msgs::Bool flag_AEB;

void UltarSonarCallback_Range(const sensor_msgs::Range::ConstPtr& msg)
{
	ROS_INFO("Soner Seq: [%d]", msg->header.seq);
	ROS_INFO("Soner Range: [%f]", msg->range);	
}

void UltraSonarCallback_Bool(const std_msgs::Bools::ConstPtr& bool) 
{
	if(bool == true)
	{
		ROS_INFO("AEB_Activated");
		flag_AEB.data = true;
	}
	else {
		flag_AEB.data = false;
	}
}

int main(int argc, char **argv) {
	int count = 0;
	
	ros::init(argc, argv, "aeb_controller_sub");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);
	ros::Subscriber sub1 = n.subscribe("bool", 1000, UltraSonarCallback_Bool);
	ros::Subscriber sub2 = n.subscribe("range", 1000, UltraSonarCallback_Range);
	
	while (ros::ok()) {
		
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}
	return 0;
}
