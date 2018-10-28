#include <iostream>
#include "ros/ros.h"
#include <fstream>
#include <math.h>
#include "mavrosCommand.hpp"
#include <nlohmann/json.hpp>
#include <wiringPi.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "rplidar.h"

using namespace std;

double latitude[65];
double longitude[65];
int pointsCount = 0;
float missionAltitude = 0.5;
int yawMapping;
#define PI 3.14159265

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

enum directions
{
	FromStartLine = 1,
	ToStartLine = 2
};

using json = nlohmann::json;

const uint8_t WAIT_FOR_START = 0;
const uint8_t TAKEOFF_HOME = 1;
const uint8_t NEXT_POINT = 2;
const uint8_t FLY_HOME = 3;
const uint8_t LAND_HOME = 4;
const uint8_t END = 5;
int i = 0;
int currentState = 0;
bool isInit = false;
double homeLatitude;
double homeLongitude;
double homeAltitude;
bool precisionLanding = false;
double dronAltitude;


//Czestotliwosc do ustawienia w Hz
int frequency = 20;
//////////////////////////
int loopCounter;
int loopCounter1;

int checkpointsQuantity = 11;
double cordinatesPrecision = 0.00002;//0.000005;
//////////////////

void mission(mavrosCommand command);
void waitForStart(mavrosCommand command);
void takeOffHome(mavrosCommand command);
void nextPoint(mavrosCommand command);
void flyHome(mavrosCommand command);
void landHome(mavrosCommand command);
bool getCordinates(mavrosCommand command);


using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

int main(int argc, char* argv[])
{
	cout<<"initializaiton"<<endl;
	ros::init(argc, argv, "rescue");
	mavrosCommand command;
	ros::Rate loop_rate(10);
	sleep(1);

    u_result     op_result;
    printf("RPLIDAR Version: " RPLIDAR_SDK_VERSION "\n");

    // create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv)
    {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
    
    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    // make connection...
    
    if(!drv)
    {
		drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
	}
	
	if (IS_OK(drv->connect("/dev/ttyUSB0", 256000)))
	{
		op_result = drv->getDeviceInfo(devinfo);

		if (IS_OK(op_result)) 
		{
			connectSuccess = true;
		}
		else
		{
			delete drv;
			drv = NULL;
		}
	}

    if (!connectSuccess) 
    {    
        fprintf(stderr, "Error, cannot bind to the specified serial port /dev/ttyUSB0.\n");
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        goto on_finished;
    }
    
    printf("Starting RPLIDAR motor.\n");
    drv->startMotor();
	printf("Starting RPLIDAR scaning.\n");
    drv->startScan(0,1);

    // fetech result and print it out...
    while (ros::ok()) {
        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);

        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
           /* for (int pos = 0; pos < (int)count ; ++pos) {
                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
                    (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                    nodes[pos].distance_q2/4.0f,
                    nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            }*/
            
            printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                (nodes[0].quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ", 
                (nodes[0].angle_z_q14 >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
                nodes[0].dist_mm_q2 / 4.0f,
                nodes[0].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            
        }
		
		ros::spinOnce();
		loop_rate.sleep();
    }

	printf("Stoping RPLIDAR motor.\n");
    drv->stop();
    drv->stopMotor();
    
    // done!
on_finished:
	printf("Disposing RPLIDAR driver.\n");
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;

	/*ros::init(argc, argv, "beginner_tutorials");
	mavrosCommand command;
	
	ros::Rate loop_rate(frequency);
	sleep(1);
	
	
	

	if(getCordinates(command) == false){
		cout<<"FILE mission.txt IS DAMAGED!"<<endl;
		return 0;
	}*/
	/*
	for(i=0; i<ile; i++){
		cout<<fixed << setprecision(7) << latitude[i] <<", ";
		cout<<fixed << setprecision(7) << longitude[i] <<endl;
	}
	*/
	/*i=0;
	
	while (ros::ok()) {
		
		if(loopCounter >= 10){
			mission(command);
			loopCounter = 0;
		}
		
		loopCounter++;
		loopCounter1++;
		ros::spinOnce();
		loop_rate.sleep();
	}	
*/	
	return 0;
}

void mission(mavrosCommand command){
	switch(currentState){
		case WAIT_FOR_START:
			if(isInit == true)waitForStart(command);
			else{
				command.initSubscribers();
				isInit = true;
			}
		break;
		case TAKEOFF_HOME:
			takeOffHome(command);
		break;
		case NEXT_POINT:
			nextPoint(command);
		break;
		case LAND_HOME:
			landHome(command);
		break;
		case END:
			cout<<"END OF MISSION"<<endl;
			exit(0);
		break;
		default:

		break;
	}
}

void waitForStart(mavrosCommand command){
	
	homeLatitude = command.getGlobalPositionLatitude();
	homeLongitude = command.getGlobalPositionLongitude();
	homeAltitude = command.getGlobalPositionAltitude();
	
	latitude[pointsCount] = homeLatitude;
	longitude[pointsCount] = homeLongitude;
	
	dronAltitude = missionAltitude;
	command.guided();
	sleep(1);

	command.arm();
	sleep(1);

	command.takeOff(missionAltitude);
	currentState = TAKEOFF_HOME;
	sleep(3);
	command.flyTo(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), missionAltitude);
	
}

void takeOffHome(mavrosCommand command){
	
	cout<<"CURRENT ALTITUDE: "<< command.getGlobalPositionAltitude() - homeAltitude <<endl;
	if(command.getGlobalPositionAltitude() - homeAltitude >= missionAltitude){
		currentState = NEXT_POINT;
		command.flyTo(latitude[i], longitude[i], missionAltitude);
		dronAltitude = missionAltitude;
		cout<<"RIGHT ALTITUDE"<<endl;
		cout<<"FLY DESTINATION: ";
		cout<<fixed << setprecision(7) << latitude[i] <<", ";
		cout<<fixed << setprecision(7) << longitude[i] <<endl;
	
	}
	else{
		 dronAltitude = dronAltitude + 0.01;
		 command.flyTo(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), dronAltitude);
	 }
}

void nextPoint(mavrosCommand command){
	
	cout<<"CURRENT POSITION: ";
	cout<<fixed << setprecision(7) << command.getGlobalPositionLatitude()<<", ";
	cout<<fixed << setprecision(7) << command.getGlobalPositionLongitude()<<" ";
	cout<<" CURRENT ALTITUDE: ";
	cout<< command.getGlobalPositionAltitude() - homeAltitude<<endl;
	
	if(command.isInPosition(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), latitude[i], longitude[i], cordinatesPrecision)){
		
		i++;
		
		if(i >= pointsCount + 1){ //IS IN HOME POSITION?
			currentState = LAND_HOME;
			dronAltitude = 5;
			command.flyTo(homeLatitude, homeLongitude, dronAltitude);
			return;
		}
		
		command.flyTo(latitude[i], longitude[i], missionAltitude);
	}
}

void landHome(mavrosCommand command){
	
	if(!precisionLanding){
		
		cout<<"CURRENT ALTITUDE: "<< command.getGlobalPositionAltitude() - homeAltitude<<endl;	
		if(command.getGlobalPositionAltitude() - homeAltitude <= 5){
			precisionLanding = true;
			sleep(3);
			command.land();
		}
		else{
			 dronAltitude = dronAltitude - 0.1;
			 command.flyTo(homeLatitude, homeLongitude, dronAltitude);
		 }
	}
	else{
		cout<<command.getArmed()<<endl;
		if(!command.getArmed())currentState = END;
	}
}

bool getCordinates(mavrosCommand command){
	ifstream theFile("/home/w0rt4/drony/catkin_ws/src/mission2/mission.json");
	json missionSettings = json::parse(theFile);
	theFile.close();
	
	int ik,jk;
 	double x, x_wsp_14, x_wsp_12, x_pom;
 	double y, y_wsp_14, y_wsp_12, y_pom;
 	double easting, northing, longitudeShift, latitudeShift;
 	int zone;
 	bool northp;
 	
 	missionAltitude = missionSettings["mission"]["altitude"];
 	
 	double leftDownLongitude = missionSettings["mission"]["leftDown"]["longitude"];
 	double leftDownLatitude = missionSettings["mission"]["leftDown"]["latitude"];
 	double leftUpLongitude = missionSettings["mission"]["leftUp"]["longitude"];
 	double leftUpLatitude = missionSettings["mission"]["leftUp"]["latitude"];
 	double rightDownLongitude = missionSettings["mission"]["rightDown"]["longitude"];
 	double rightDownLatitude = missionSettings["mission"]["rightDown"]["latitude"];
 	
 	int direction = directions(FromStartLine);
 	
 	int pointsOnSingleScan = 8;
 	int scanCount = 8;
 	
	if(missionAltitude == 0 || leftDownLatitude == 0 || leftUpLatitude == 0 || rightDownLatitude == 0 || leftDownLongitude == 0 || leftUpLongitude == 0 || rightDownLongitude == 0)
	{
		return false;
	}	 
	
	yawMapping = atan((leftUpLongitude - leftDownLongitude) * 0.67 / (leftUpLatitude - leftDownLatitude) * 1.11) * 180 / PI;

	if(leftUpLongitude - leftDownLongitude >= 0 && leftUpLatitude - leftDownLatitude == 0)
	{
		yawMapping = 90;
	}
	else if(leftUpLatitude - leftDownLatitude < 0)
	{
		yawMapping = 180 + yawMapping;
	}
	else if(leftUpLongitude - leftDownLongitude < 0  && leftUpLatitude - leftDownLatitude == 0)
	{
		yawMapping = 270;
	}
	else if(leftUpLongitude - leftDownLongitude < 0  && leftUpLatitude - leftDownLatitude > 0)
	{
		yawMapping = 360 + yawMapping;
	}
	
	yawMapping = yawMapping % 360;
				
 	x_wsp_12 = leftUpLatitude - leftDownLatitude;
	y_wsp_12 = leftUpLongitude - leftDownLongitude;
	x_wsp_14 = rightDownLatitude - leftDownLatitude;
	y_wsp_14 = rightDownLongitude - leftDownLongitude;
	x_wsp_12 = x_wsp_12 / (pointsOnSingleScan - 1);
	y_wsp_12 = y_wsp_12 / (pointsOnSingleScan - 1);
	x_wsp_14 = x_wsp_14 / (scanCount - 1);
	y_wsp_14 = y_wsp_14 / (scanCount - 1);
	x_pom = leftDownLatitude;
	y_pom = leftDownLongitude;
 	
 	for(jk = 0; jk < scanCount; jk++)
 	{
		if(direction == directions(FromStartLine))
		{
			x = x_pom;
			y = y_pom;
			latitude[pointsCount] = x;
			longitude[pointsCount] = y;
			pointsCount++;
			
			for(ik = 1; ik < pointsOnSingleScan; ik++)
			{
				x = x_pom + ik * x_wsp_12;
				y = y_pom + ik * y_wsp_12;
				latitude[pointsCount] = x;
				longitude[pointsCount] = y;
				pointsCount++;
			}
			
			x_pom = x + x_wsp_14;
			y_pom = y + y_wsp_14;
			
			direction = directions(ToStartLine);
	   }
		else if(direction == directions(ToStartLine))
		{
			x = x_pom;
			y = y_pom;
			latitude[pointsCount] = x;
			longitude[pointsCount] = y;
			pointsCount++;
			
			for(ik = 1; ik < pointsOnSingleScan; ik++)
			{
				x = x_pom - ik * x_wsp_12;
				y = y_pom - ik * y_wsp_12;
				latitude[pointsCount] = x;
				longitude[pointsCount] = y;
				pointsCount++;
			}
			
			x_pom = x + x_wsp_14;
			y_pom = y + y_wsp_14;
			direction = directions(FromStartLine);
	   }
	}
	
	return true;
}


