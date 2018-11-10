#include "flyController.hpp"
#include <chrono>

using namespace rp::standalone::rplidar;

void FlyController::sendCommandToDron(mavrosCommand command, rplidar_response_measurement_node_hq_t nodes[8192], size_t count, target Target)
{
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
}

FlyController::moveDroneCommand FlyController::decideWhereToFly(mavrosCommand command, rplidar_response_measurement_node_hq_t nodes[8192], size_t count, target Target)
{
	double bearingToTarget = command.getBearingBetweenCoordinates(command.getGlobalPositionLatitude(), command.getGlobalPositionLatitude(), Target.Latitude, Target.Longitude);
	double angleFromDronePerspetive = fmod(0 - (command.getCompassHeading() - bearingToTarget) + 360, 360); 
	FlyController::moveDroneCommand droneCommand;
	int fake;
	
	/*for(int i = 0; i < count; i++)
	{
		cout << "S nr:"<< i<<" angle:"<<nodes[i].angle_z_q14 * 90.f / (1 << 14)<<" distance: "<<((nodes[i].dist_mm_q2 / 4.0f))<<endl;
	}
	
	cout<<"count: "<<count<<endl;*/
	
	for(int x = 1; x <= 36; x++)
	{
		for(int i = 0; i < count; i++)
		{
			if((fmod(((nodes[i].angle_z_q14 * 90.f / (1 << 14)) - fmod((angleFromDronePerspetive - 5 * x) + 360, 360)) + 360, 360) <= 5 * x
			|| fmod(fmod((angleFromDronePerspetive + 5 * x) + 360, 360) - (nodes[i].angle_z_q14 * 90.f / (1 << 14)) + 360, 360) <= 5 * x)
			&& (nodes[i].dist_mm_q2 / 4.0f) >= 1200
			&& (nodes[i].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) != 0)
			{
				bool hasSpace= true;
				for(int j = 0; j < 8192; j++)
				{
					if((fmod(((nodes[j].angle_z_q14 * 90.f / (1 << 14)) - ((nodes[i].angle_z_q14 * 90.f / (1 << 14)) - 15)) + 360, 360) <= 15
					|| fmod((((nodes[i].angle_z_q14 * 90.f / (1 << 14)) + 15) - (nodes[j].angle_z_q14 * 90.f / (1 << 14))) + 360, 360) <= 15)
					&& (nodes[j].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) != 0)
					{
						if((nodes[j].dist_mm_q2 / 4.0f) < 1200)
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
						droneCommand.Yaw = direction - 180;
					}
					else
					{
						droneCommand.Yaw = -direction;
					}
					
					droneCommand.CanBeStoped = false;
					droneCommand.IsCorrect = true;
					//cin >> fake;
					return droneCommand;
				}
			}
		}
	}
	
	droneCommand.IsCorrect = false;	
	//cin >> fake;
	return droneCommand;
}
