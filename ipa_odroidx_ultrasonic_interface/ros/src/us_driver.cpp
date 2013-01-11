#include <stdio.h>
#include <vector>
#include "ros/ros.h"
#include "ipa_odroidx_ultrasonic_interface/UARTDriver.h"

#define PINGING_SENSOR 	-1
#define SENSOR_NOT_USED 255

/*generateConfigVector stores the pinging sensors, to which pinging sensor is a listening
 * sensor bound to in a cycle and which sensors are not used at all.
 * In the output, the pinging sensors are represented by PINGING_SENSOR, location of
 * each listening sensor holds the address of its corresponding pinging sensor
 * and the sensors not used at all are represented by SENSOR_NOT_USED.
 */
std::vector <std::vector<int> > generateConfigVector(XmlRpc::XmlRpcValue config_list)
{
	std::vector<std::vector<int> > config;
	XmlRpc::XmlRpcValue current_cycle;
	ROS_ASSERT(config_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for(int i = 0; i < config_list.size(); i++)
	{
		ROS_ASSERT(config_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
		std::vector<int> cycle_vector; //For holding configuration for each cycle.
		cycle_vector.resize(14);
		for (int j=0; j<14; j++)
			cycle_vector[j]=SENSOR_NOT_USED;

		for (int j = 0; j<14; j++){
			char str_index[3] = {'\0','\0', '\0'}; // for storing int to hex conv. 
			sprintf(str_index,"%d", j); // Unsure if yamlcpp parser reads hexadecimal values
			if(config_list[i].hasMember(str_index))
			{
				cycle_vector[j]=PINGING_SENSOR; //Set -1 for pinging sensor
				current_cycle = config_list[i][str_index];
				ROS_ASSERT(current_cycle.getType() == XmlRpc::XmlRpcValue::TypeArray);
				ROS_ASSERT(current_cycle.size()>0);
				//Assign address of pinging sensor to each listening sensor
				for (int k = 0; k<current_cycle.size(); k++){
					if (static_cast<int>(current_cycle[k])!=j)
						cycle_vector[static_cast<int>(current_cycle[k])]=j;
				}
			}
		}
		config.push_back(cycle_vector);
	}
	return config;
}
int generateConfigString(std::vector< std::vector<int> >config_vector,unsigned char * config_string)
{
	unsigned char temp_config_string[100];
	int config_string_length=config_vector.size()*2+1; //setting up length of config string

	if(config_vector.size()==0)
		return 0;
	temp_config_string[0]=(unsigned int)(config_vector.size()) & 0xff;
	int count=1;
	for (std::vector< std::vector<int> >::iterator list_it = config_vector.begin(); list_it != config_vector.end(); list_it++)
	{
		int temp_mask=0;
		if((*list_it).size()==0)
			return 0;
		for (int i=0; i<14; i++)
		{
			if((*list_it)[i]==PINGING_SENSOR)
			{
				if (i<7)
					temp_mask|=(1<<i); //For first port
				else
					temp_mask|=(1<<(i+1)); // For second port
			}
		}
		temp_config_string[count++] = (temp_mask & 0xff00) >> 8;
		temp_config_string[count++] = (temp_mask & 0xff);
	}
	memcpy(config_string, temp_config_string, config_string_length); // copying contents of temp config onto config string before return.
	return config_string_length;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "us_driver");
	ros::NodeHandle nh_;
	XmlRpc::XmlRpcValue config_list_;
	ros::init(argc, argv, "us_driver");
	if(!nh_.hasParam("us_driver/configurations"))
	{
		ROS_ERROR("Sensor configurations not found.");
		return(EXIT_FAILURE);
	}
	ROS_INFO("configurations found.");

	CommPortDriver * comm_port = new UARTDriver("/dev/ttyUSB0");

	nh_.getParam("us_driver/configurations", config_list_);
	ROS_ASSERT(config_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);

	std::vector <std::vector<int> > config_vector_ = generateConfigVector(config_list_);
	ROS_INFO("%d", (int)config_vector_.size());
	for (int i=0; i<14; i++){
		ROS_INFO("%d", config_vector_[0][i]);
	}

	unsigned char config_string_[100]; // Must be declared before use.
	int config_string_length_ = 0;

	config_string_length_ = generateConfigString(config_vector_, config_string_);
	ROS_INFO("config_string_length_ = %d", config_string_length_);
	for (int i= 0; i<config_string_length_; i++)
	{
		ROS_INFO("0x%02x", config_string_[i]);
	}

	comm_port->writeBytes(config_string_, config_string_length_);
	unsigned char * buffer = new unsigned char[100];
	while(ros::ok())
	{
		comm_port->readBytes(buffer, 1);
		ROS_INFO("%02x", buffer[0]);
	}
	delete comm_port;
	return 0;
}
