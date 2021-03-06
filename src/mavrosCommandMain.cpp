#include <iostream>
#include <fstream>
#include <math.h>
#include <nlohmann/json.hpp>
#include <wiringPi.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "rplidar.h"
#include "radarController.hpp"
#include "flyController.hpp"
#include "mavrosCommand.hpp"

using json = nlohmann::json;

using namespace std;

struct home 
{
	double Latitude;
	double Longitude;
	double Altitude;
} Home;

FlyController::target Target;
FlyController::target Start;

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
	FlyController flyController;
	
	
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
    missionStatus = StartRescue;
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
				break;
			case TakeOff:
				if(takeOffHome(command))
				{
					missionStatus = FlyToStart;
				}
				break;
			case FlyToStart:
				if(flyToStartPosition(command))
				{
					missionStatus = StartRescue;
				}
				break;
			case StartRescue:
				if(command.isInPosition(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), Target.Latitude, Target.Longitude, 0.00006))
				{
					missionStatus = CallForPackage;
				}
				else
				{
					flyController.sendCommandToDron(command, radarController, Target);
				}
				break;
			case CallForPackage:
				if(flyController.searchForTarget(command, radarController, rsImage))
				{
					missionStatus = BackToStart;
				}
				break;
			case BackToStart:
				if(command.isInPosition(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), Start.Latitude, Start.Longitude, 0.00006))
				{
					missionStatus = FlyToHome;
				}
				else
				{
					flyController.sendCommandToDron(command, radarController, Start);
				}
				break;
			case FlyToHome:
				if(flyHome(command))
				{
					missionStatus = LandHome;
				}
				break;
			case LandHome:
				if(landHome(command))
				{
					missionStatus = End;
				}
				break;
			default:
				cout << "MISSION END!" << endl;
				return 0;
				break;
		}

		/*if(counter>=20)
		{
			cv_bridge::CvImageConstPtr cv_ptr = rsImage.getpicture();
			cv::Mat frame = cv_ptr->image;
			cv::imshow( "Display window", frame );
			cv::waitKey(10);
			
			counter = 0;
		}*/
		
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
	
	ifstream theFile("/home/w0rt4/drony/catkin_ws/src/Poliburdel_Zad3/mission.json");
	json missionSettings = json::parse(theFile);
	theFile.close();
 	
 	Mission.MissionAltitude = missionSettings["mission"]["altitude"];
 	
 	Start.Longitude = missionSettings["mission"]["start"]["longitude"];
 	Start.Latitude = missionSettings["mission"]["start"]["latitude"];
 	Target.Longitude = missionSettings["mission"]["target"]["longitude"];
 	Target.Latitude = missionSettings["mission"]["target"]["latitude"];
 	
	return true;
}
