#include <stdio.h>
#include <vector>
#include "ros/ros.h"
#include "ipa_odroidx_ultrasonic_interface/UARTDriver.h"

#define PINGING_SENSOR 	-1
#define SENSOR_NOT_USED 255
#define MAX_SENSORS 14

enum ACK_RECEIVED {NO, MAYBE, YES}; // Three states of ACK 

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
		cycle_vector.resize(MAX_SENSORS);
		for (int j=0; j<MAX_SENSORS; j++)
			cycle_vector[j]=SENSOR_NOT_USED;

		for (int j = 0; j<MAX_SENSORS; j++){
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
		for (int i=0; i<MAX_SENSORS; i++)
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
	std::map<int, std::vector< int > > input_map_;
	ros::init(argc, argv, "us_driver");
	if(!nh_.hasParam("us_driver/configurations"))
	{
		ROS_ERROR("Sensor configurations not found.");
		return(EXIT_FAILURE);
	}
	ROS_INFO("configurations found.");

	CommPortDriver * comm_port_ = new UARTDriver("/dev/ttyUSB0");

	nh_.getParam("us_driver/configurations", config_list_);
	ROS_ASSERT(config_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);

	std::vector <std::vector<int> > config_vector_ = generateConfigVector(config_list_);
	ROS_INFO("%d", (int)config_vector_.size());
	for (int i=0; i<MAX_SENSORS; i++){
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
	ROS_INFO(" ");
	ROS_ASSERT(comm_port_->writeBytes(config_string_, config_string_length_)==config_string_length_);
	unsigned char * buffer_ = new unsigned char[100];

	ACK_RECEIVED ack_received_ = NO;
	bool ack_stage_2 = false;
	int sequence_number= -1;
	while(ros::ok())
	{
		comm_port_->readBytes(buffer_, 1);
		ROS_INFO("%02x", buffer_[0]);
		if(ack_received_ == NO)
		{
			ROS_INFO("ACK NO");
			if(buffer_[0] == 0x12)
				ack_received_ = MAYBE;
		}
		else if(ack_received_ == MAYBE)
		{
			ROS_INFO("ACK MAYBE");
			if(ack_stage_2 == false)
			{
				if (buffer_[0] == 0x00)
				{
					ROS_INFO("ack_stage_2 = true");
					ack_stage_2 = true;
					sequence_number = 0;
				}
				else
					ack_received_ = NO;
			}
			else if ((buffer_[0] & 0xf0) == 0xd0)
			{
				ack_received_ = YES;
			}
		}
		if(ack_received_==YES) //ack reception confirmed.
		{
//			ROS_INFO("ACK YES");
//			ROS_INFO("0x%02x", buffer_[0]);
			if(sequence_number == -1) //previous cycle complete.
			{
				sequence_number = buffer_[0];
				ROS_INFO("sequence_number: 0x%x", sequence_number);
			}
			else
			{
				for (int i=0; i< MAX_SENSORS; i++)
				{
					if (i>0){
						comm_port_->readBytes(buffer_,1);
					}
					int total_sensor_readings = (buffer_[0] & 0x0f) ;
//					ROS_INFO("sensor address: %x, total_sensor_readings: %d",(buffer_[0] & 0xf0) >> 4, total_sensor_readings);
					if(total_sensor_readings > 0)
					{
						int current_sensor_address = (buffer_[0] & 0xf0) >> 4;
						for (int j = 0; j < total_sensor_readings; j++)
						{
							int temp_reading = 0;
							comm_port_->readBytes(buffer_, 1);
							temp_reading= (buffer_[0]<<8 & 0xff00);
							comm_port_->readBytes(buffer_, 1);
							temp_reading |= (buffer_[0] & 0xff);
							input_map_[current_sensor_address].push_back(temp_reading & 0xffff);
//							ROS_INFO("Readings read: %d", input_map_[current_sensor_address].size());
						}
					}
				}
				for (std::map<int, std::vector<int> >::iterator map_it = input_map_.begin(); map_it != input_map_.end(); map_it++)
				{
					ROS_INFO("Sensor address: %d", map_it->first);
					ROS_INFO("----");
					for (std::vector<int>::iterator reading_list_it = (*map_it).second.begin(); reading_list_it != (*map_it).second.end(); reading_list_it++)
					{
						ROS_INFO("0x%04x", (*reading_list_it) & 0xffff);
					}
					ROS_INFO("----");
				}


				//To indicate that one complete cycle has been processed.
				input_map_.clear();
				sequence_number = -1;
			}
		}
	}
	delete comm_port_;
	return 0;
}
