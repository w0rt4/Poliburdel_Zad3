#include "flyController.hpp"

using namespace rp::standalone::rplidar;

void FlyController::sendCommandToDron(mavrosCommand command, RadarController radarController, target Target)
{
	switch (instruction)
	{
		case Turning:
			if (isTurning)
			{
				if (abs(command.getCompassHeading() - targetAngle) <= 2)
				{
					isTurning = false;
					targetAngle = 0;
					
					instruction = Moving;
				}
			}
			else
			{
				rplidar_response_measurement_node_hq_t nodes[8192];
				size_t count;
				radarController.GetNodes(nodes, count); 
					
				auto begin = chrono::high_resolution_clock::now();
				moveDroneCommand droneCommand = decideWhereToFly(command, nodes, count, Target);
				auto end = chrono::high_resolution_clock::now();
				cout << "Time spend in function: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()<<"ms"<<endl;
				if(droneCommand.IsCorrect)
				{
					cout<<"Diretion is: "<<droneCommand.Yaw<<endl;
				}
				else
				{
					cout<<"Error when decide to fly. No command was send"<<endl;
				}
				
				targetAngle = droneCommand.TargetBearing;
				isTurning = true;
				command.flyToLocal(0, 0, 0, droneCommand.Yaw);
			}
			break;
		case Moving:
			if(isMoving)
			{
				auto now = chrono::high_resolution_clock::now();
				if(std::chrono::duration_cast<std::chrono::milliseconds>(now - movingStartTime).count() >= 2000)
				{
					isMoving = false;
					instruction = Turning;
				}
			}
			else
			{
				command.flyToLocal(1, 0, 0, 0);
				movingStartTime = chrono::high_resolution_clock::now();
				isMoving = true;
			}
			break;
	}
}

FlyController::moveDroneCommand FlyController::decideWhereToFly(mavrosCommand command, rplidar_response_measurement_node_hq_t nodes[8192], size_t count, target Target)
{
	double bearingToTarget = command.getBearingBetweenCoordinates(command.getGlobalPositionLatitude(), command.getGlobalPositionLatitude(), Target.Latitude, Target.Longitude);
	double angleFromDronePerspetive = fmod(0 - (command.getCompassHeading() - bearingToTarget) + 360, 360); 
	FlyController::moveDroneCommand droneCommand;
	
	for(int x = 1; x <= 36; x++)
	{
		for(int i = 0; i < count; i++)
		{
			if((fmod(((nodes[i].angle_z_q14 * 90.f / (1 << 14)) - fmod((angleFromDronePerspetive - 5 * x) + 360, 360)) + 360, 360) <= 5 * x
			|| fmod(fmod((angleFromDronePerspetive + 5 * x) + 360, 360) - (nodes[i].angle_z_q14 * 90.f / (1 << 14)) + 360, 360) <= 5 * x)
			&& ((nodes[i].dist_mm_q2 / 4.0f) >= 2000
			|| (nodes[i].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) == 0))
			{
				bool hasSpace= true;
				for(int j = 0; j < 8192; j++)
				{
					if((fmod(((nodes[j].angle_z_q14 * 90.f / (1 << 14)) - ((nodes[i].angle_z_q14 * 90.f / (1 << 14)) - 15)) + 360, 360) <= 15
					|| fmod((((nodes[i].angle_z_q14 * 90.f / (1 << 14)) + 15) - (nodes[j].angle_z_q14 * 90.f / (1 << 14))) + 360, 360) <= 15))
					{
						if((nodes[j].dist_mm_q2 / 4.0f) < 2000	&& (nodes[j].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) != 0)
						{
							hasSpace = false;
							break;
						}
					}
					else 
					{
						continue;
					}
				}
				
				if(hasSpace)
				{
					float direction = nodes[i].angle_z_q14 * 90.f / (1 << 14);
					
					if(direction > 180)
					{
						droneCommand.Yaw = 360 - direction;
					}
					else
					{
						droneCommand.Yaw = -direction;
					}
					
					droneCommand.TargetBearing = bearingToTarget;
					droneCommand.IsCorrect = true;
					
					return droneCommand;
				}
			}
		}
	}
	
	droneCommand.IsCorrect = false;	
	if(angleFromDronePerspetive > 180)
	{
		droneCommand.Yaw = 360 - angleFromDronePerspetive;
	}
	else
	{
		droneCommand.Yaw = -angleFromDronePerspetive;
	}

	return droneCommand;
}

bool FlyController::searchForTarget(mavrosCommand command, RadarController radarController)
{
	if(scanCount > 13)
	{
		// wzywamy malego na obecna pozycje drona
		return true;
	}
	
	if(isScaning)
	{
		if (abs(command.getCompassHeading() - targetAngle) <= 2)
		{
			scanCount++;
			isScaning = false;
			
			// poszukiwanie goscia z kamery
			//jak sie udalo to wzywamy malego i return true
		}
	}
	else
	{
		targetAngle = command.getCompassHeading() + 20;
		command.flyToLocal(0, 0, 0, -20);
	}
	
	return false;
}
