if (nh_.hasParam("sensors"))
			{
	XmlRpc::XmlRpcValue v;
    nh_.param("sensors", v, v);
    for(int i =0; i < v.size(); i++)
    {
	ROS_ASSERT(v[i].size()>=2);

	int id = v[i][0];
	std::string fr_id = v[i][1];
	int filter = v.size()>2?(int)v[i][2]:10;

	g_sensors.push_back(Sensor(fr_id,id,filter));
    }
			}
			else
			{
				ROS_ERROR("Parameter sensors not set, shutting down node...");
				nh_.shutdown();
				return false;
			}
