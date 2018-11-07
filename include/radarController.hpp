#pragma once

#include <iostream>
//#include "ros/ros.h"
#include <fstream>
#include <math.h>
//#include "mavrosCommand.hpp"
#include "rplidar.h"

using namespace rp::standalone::rplidar;

class RadarController
{
	public:
		RadarController();
		~RadarController();
		bool ConnectionSuccess;
		void GetNodes(rplidar_response_measurement_node_hq_t (&nodes)[8192]);
		void Initialize();
		bool CheckRPLIDARHealth();
		void PrintInfo();
		void StartScan();
		void EndScan();
	
	private:
		RPlidarDriver * drv;
		u_result op_result;
		rplidar_response_device_info_t devinfo;
};
