#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"

std_msgs::Bool msg;
sensor_msgs::Range range;

int main(int argc, char **argv) {
	int count = 0;
	
	ros::init(argc, argv, "aeb_controller_pub");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);
	ros::Publisher pub1 = n.advertise<std_msgs::Bool>("bool", 1000);
	ros::Publisher pub2 = n.advertise<sensor_msgs::Range>("range", 1000);

	if(range.range <= 1.0) {
		msg.data = true;
	}

	else {
		msg.data = false;
	}


	while (ros::ok()) {
		pub1.publish(msg);
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}
	return 0;
}
