#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "ipa_odroidx_ultrasonic_interface/ExRange.h"
#include "ipa_odroidx_ultrasonic_interface/ExRangeArray.h"
#include <vector>
#include <stdlib.h>

ros::Publisher publisher;
void MakeExRangeArray(const sensor_msgs::Range range_msg)
{
	static std::vector<ipa_odroidx_ultrasonic_interface::ExRange> ExRangeVector;
	static bool in_cycle = false;
	if (in_cycle!=true)
	{
		if (range_msg.header.frame_id == "/sonar_1_link")
			in_cycle = true;
	}
	if(in_cycle==true){
		ipa_odroidx_ultrasonic_interface::ExRange temp_exrange;
		temp_exrange.sender_ch=range_msg.header.frame_id[7]-'0';
		temp_exrange.receiver_ch=temp_exrange.sender_ch;
		temp_exrange.measurement.radiation_type = 0;
		temp_exrange.measurement.field_of_view = range_msg.field_of_view;
		temp_exrange.measurement.min_range = range_msg.min_range;
		temp_exrange.measurement.max_range = range_msg.max_range;
		temp_exrange.measurement.range = range_msg.range;

		temp_exrange.measurement.header.stamp = ros::Time::now();
		temp_exrange.measurement.header.frame_id ="us"+range_msg.header.frame_id.substr(7,1);
		ExRangeVector.push_back(temp_exrange);
	}
	if (in_cycle == true && range_msg.header.frame_id == "/sonar_8_link")
	{
		ipa_odroidx_ultrasonic_interface::ExRangeArray ex_range_array;
		for (std::vector<ipa_odroidx_ultrasonic_interface::ExRange>::iterator exrange_iter = ExRangeVector.begin(); exrange_iter != ExRangeVector.end(); exrange_iter++)
		{
			ex_range_array.measurements.push_back(*exrange_iter);
		}
		in_cycle = false;
		publisher.publish(ex_range_array);
		ExRangeVector.clear();
	}
	
}
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "range_to_exrange");
	ros::NodeHandle nh_;
	publisher = nh_.advertise<ipa_odroidx_ultrasonic_interface::ExRangeArray>("ultrasonic_msgs", 1000);
	ROS_INFO("sensor_msgs/Range to ipa_odroidx_ultrasonic_interface/ExRangeArray conversion started.");
	ros::Subscriber sub = nh_.subscribe("/sonar", 1000, MakeExRangeArray);
	ros::spin();
	return 0;
}
