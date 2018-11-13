#pragma once

#include <iostream>
#include "ros/ros.h"
#include <fstream>
#include <math.h>
#include "mavrosCommand.hpp"
#include "rplidar.h"

using namespace rp::standalone::rplidar;

class FlyController
{
	public:
	
		struct moveDroneCommand
		{
			double Forward;
			double Right;
			double Up;
			float Yaw;
			bool IsCorrect;
			float TargetBearing;
		};
		
		struct target
		{
			double Latitude;
			double Longitude;
		};
		
		void sendCommandToDron(mavrosCommand command, rplidar_response_measurement_node_hq_t nodes[8192], size_t count, target Target);
		moveDroneCommand decideWhereToFly(mavrosCommand command, rplidar_response_measurement_node_hq_t nodes[8192], size_t count, target Target);
		
	private:
	enum instructionType
	{
		Turning = 1,
		Moving = 2,
	} instruction = Turning;
	bool isTurning = false;
	float targetAngle;
};
