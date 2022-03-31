#include "ros/ros.h"
#include "sensor_msgs/Range.h"

using namespace ros;

sensor_msgs::Range range;

void RangeCallBack(const sensor_msgs::Range::ConstPtr& msg) {
	
	float r1,r2,r3,r4,r5;
	r1=r2=r3=r4=r5=msg->range;
	float sum = r1+r2+r3+r4+r5;
	
	ROS_INFO("aver = %f", sum/5);
}


int main(int argc, char **argv) {
	init(argc, argv, "sonar_avg_filter");
	NodeHandle n;
	Subscriber sub = n.subscribe("/range", 1000, RangeCallBack);
	Publisher pub_avg_range = n.advertise<sensor_msgs::Range>("/range_avg", 1000);
	
	Rate loop_rate(10);
	
	while(ok()) 
	{
		pub_avg_range.publish(range);
		
		loop_rate.sleep();
		spinOnce();
	}
	
	return 0;
	
}
