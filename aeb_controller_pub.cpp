#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"

std_msgs::Bool flag_AEB;

/*void UltraSonarCallback(const sensor_msgs::Range::ConstPtr& msg) 
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
}*/

int main(int argc, char **argv) {
	int count = 0;
	
	ros::init(argc, argv, "aeb_controller_pub");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	ros::Publisher pub = n.advertise<sensor_msgs::Range>("range", 1000);
	
	while (ros::ok()) {
		
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}
	return 0;
}
