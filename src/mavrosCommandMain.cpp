#include <iostream>
#include "ros/ros.h"
#include <fstream>
#include <math.h>
#include "mavrosCommand.hpp"
#include <nlohmann/json.hpp>
#include <wiringPi.h>
#include "sensor_msgs/LaserScan.h"
#include "rplidar.h"
#include "radarController.hpp"
#include "realsenseImagetransport.hpp"

using json = nlohmann::json;

using namespace std;

struct home 
{
	double Latitude;
	double Longitude;
	double Altitude;
} Home;

struct target
{
	double Latitude;
	double Longitude;
} Target;

struct start
{
	double Latitude;
	double Longitude;
} Start;

struct mission
{
	bool PrecisionLanding = false;
	float MissionAltitude = 0.5;
	double DronAltitude;
} Mission;

enum MissionStatus
{
	Initialize = 0,
	TakeOff = 1,
	FlyToStart = 2,
	StartRescue = 3,
	CallForPackage = 4,
	BackToStart= 5,
	FlyToHome = 6,
	LandHome = 7,
	End = 8
};

bool waitForStart(mavrosCommand command);
bool takeOffHome(mavrosCommand command);
bool flyToStartPosition(mavrosCommand command);
bool flyHome(mavrosCommand command);
bool landHome(mavrosCommand command);
bool getCordinates(mavrosCommand command);
int counter = 0;


using namespace rp::standalone::rplidar;

int main(int argc, char* argv[])
{
	cout<<"initializaiton"<<endl;
	ros::init(argc, argv, "rescue");
	ros::NodeHandle nh;
	
	mavrosCommand command(&nh);
	realsenseImagetransport rsImage(&nh);
	MissionStatus missionStatus = Initialize;
	RadarController radarController;
	
	
	ros::Rate loop_rate(10);
	sleep(1);
	
	if(getCordinates(command) == false)
	{
		cout<<"FILE mission.json IS DAMAGED!"<<endl;
		return 0;
	}
	
	radarController.Initialize();

    if (!radarController.ConnectionSuccess) 
    {    
        fprintf(stderr, "Error, cannot bind to the specified serial port /dev/ttyUSB0.\n");
		return 1;
    }
    
	radarController.PrintInfo();

    // check health...
    if (!radarController.CheckRPLIDARHealth()) {
        return 1;
    }
    
    radarController.StartScan();
    
    // fetech result and print it out...
    while (ros::ok())
    {
		switch (missionStatus)
		{
			case Initialize:
				if(!waitForStart(command))
				{
					return 1;
				}
				missionStatus = TakeOff;
				continue;
				break;
			case TakeOff:
				if(takeOffHome(command))
				{
					missionStatus = FlyToStart;
				}
				continue;
				break;
			case FlyToStart:
				if(flyToStartPosition(command))
				{
					missionStatus = StartRescue;
				}
				continue;
				break;
			case StartRescue:
				continue;
				break;
			case BackToStart:
				break;
			case FlyToHome:
				if(flyHome(command))
				{
					missionStatus = LandHome;
				}
				continue;
				break;
			case LandHome:
				if(landHome(command))
				{
					missionStatus = End;
				}
				continue;
				break;
			default:
				cout << "MISSION END!" << endl;
				return 0;
				break;
		}

		if(counter>=20)
		{
			cv_bridge::CvImageConstPtr cv_ptr = rsImage.getpicture();
			cv::Mat frame = cv_ptr->image;
			cv::imshow( "Display window", frame );
			cv::waitKey(10);
			
			counter = 0;
		}
		
		
			rplidar_response_measurement_node_hq_t nodes[8192];
			radarController.GetNodes(nodes); 
			printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
				(nodes[0].quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ", 
				(nodes[0].angle_z_q14 >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
				nodes[0].dist_mm_q2 / 4.0f,
				nodes[0].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
			counter++;
			ros::spinOnce();
			loop_rate.sleep();
    }
	cv::destroyAllWindows();
	return 0;
}

bool waitForStart(mavrosCommand command)
{
	command.initSubscribers();
	
	Home.Latitude = command.getGlobalPositionLatitude();
	Home.Longitude = command.getGlobalPositionLongitude();
	Home.Altitude = command.getGlobalPositionAltitude();
	
	Mission.DronAltitude = Mission.MissionAltitude;
	if(!command.guided())
	{
		return false;
	}
	sleep(1);

	if(!command.arm())
	{
		return false;
	}
	sleep(1);

	command.takeOff(Mission.MissionAltitude);
	sleep(3);
	command.flyTo(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), Mission.MissionAltitude);

	return true;
}

bool takeOffHome(mavrosCommand command)
{	
	cout<<"CURRENT ALTITUDE: "<< command.getGlobalPositionAltitude() - Home.Altitude <<endl;
	if(command.getGlobalPositionAltitude() - Home.Altitude >= Mission.MissionAltitude)
	{
		Mission.DronAltitude = Mission.MissionAltitude;
		cout<<"RIGHT ALTITUDE"<<endl;
		cout<<"FLY DESTINATION: ";
		cout<<fixed << setprecision(7) << Start.Latitude <<", ";
		cout<<fixed << setprecision(7) << Start.Longitude <<endl;
		
		return true;
	}

	Mission.DronAltitude = Mission.DronAltitude + 0.01;
	command.flyTo(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), Mission.DronAltitude);
	return false;
}

bool flyToStartPosition(mavrosCommand command)
{
	cout<<"CURRENT POSITION: ";
	cout<<fixed << setprecision(7) << command.getGlobalPositionLatitude()<<", ";
	cout<<fixed << setprecision(7) << command.getGlobalPositionLongitude()<<" ";
	cout<<" CURRENT ALTITUDE: ";
	cout<< command.getGlobalPositionAltitude() - Home.Altitude<<endl;
	
	if(command.isInPosition(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), Start.Latitude, Start.Longitude, 0.00002))
	{
		return true;
	}
	else if(command.isInPosition(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), Home.Latitude, Home.Longitude, 0.00002))
	{
		cout<<"RESEND COMMAND FLY TO START POSITION"<<endl;
		command.flyTo(Start.Latitude, Start.Longitude, Mission.MissionAltitude);
	}
	
	return false;
}

bool flyHome(mavrosCommand command)
{
	if(command.isInPosition(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), Home.Latitude, Home.Longitude, 0.00002))
	{
		return true;
	}
	
	cout<<"RESEND COMMAND FLY TO START POSITION"<<endl;
	command.flyTo(Home.Latitude, Home.Longitude, Mission.MissionAltitude);
	
	return false;
}

bool landHome(mavrosCommand command)
{
	if(!Mission.PrecisionLanding)
	{
		cout<<"CURRENT ALTITUDE: "<< command.getGlobalPositionAltitude() - Home.Altitude<<endl;	
		if(command.getGlobalPositionAltitude() - Home.Altitude <= 5)
		{
			Mission.PrecisionLanding = true;
			sleep(3);
			command.land();
			
			return false;
		}
		
		Mission.DronAltitude = Mission.DronAltitude - 0.1;
		command.flyTo(Home.Latitude, Home.Longitude, Mission.DronAltitude);
		return false;
	}
	
	cout<<command.getArmed()<<endl;
	return true;
}

bool getCordinates(mavrosCommand command)
{
	string name = get_username();
	
	ifstream theFile("/home/" + name + "/catkin_ws/src/Poliburdel_Zad3/mission.json");
	json missionSettings = json::parse(theFile);
	theFile.close();
 	
 	Mission.MissionAltitude = missionSettings["mission"]["altitude"];
 	
 	Start.Longitude = missionSettings["mission"]["start"]["longitude"];
 	Start.Latitude = missionSettings["mission"]["start"]["latitude"];
 	Target.Longitude = missionSettings["mission"]["target"]["longitude"];
 	Target.Latitude = missionSettings["mission"]["target"]["latitude"];
 	
	return true;
}
