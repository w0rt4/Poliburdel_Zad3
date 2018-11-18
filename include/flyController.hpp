#pragma once

#include <iostream>
#include "ros/ros.h"
#include <fstream>
#include <math.h>
#include "mavrosCommand.hpp"
#include "radarController.hpp"
#include "realsenseImagetransport.hpp"
#include "rplidar.h"
#include <chrono>

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
		
		void sendCommandToDron(mavrosCommand command, RadarController radarController, target Target);
		moveDroneCommand decideWhereToFly(mavrosCommand command, rplidar_response_measurement_node_hq_t nodes[8192], size_t count, target Target);
		bool searchForTarget(mavrosCommand command, RadarController radarController, realsenseImagetransport rsImage);
		
	private:
		enum instructionType
		{
			Turning = 1,
			Moving = 2,
		} instruction = Turning;
		
		bool isScaning = false;
		int scanCount = 0;
		
		bool isTurning = false;
		bool isMoving = false;
		
		std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::duration<long int, std::ratio<1l, 1000000000l> > >  movingStartTime;
		
		float targetAngle;
};
