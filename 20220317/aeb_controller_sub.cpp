#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"

std_msgs::Bool flag_AEB;

void UltraSonarCallback_Range(const sensor_msgs::Range::ConstPtr& msg1)
{
	ROS_INFO("Soner Seq: [%d]", msg1->header.seq);
	ROS_INFO("Soner Range: [%f]", msg1->range);	
}

void UltraSonarCallback_Bool(const std_msgs::Bool::ConstPtr& msg2) 
{
	if(msg2->data == true)
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
