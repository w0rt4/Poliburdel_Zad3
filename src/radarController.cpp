#include "radarController.hpp"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;

RadarController::RadarController()
{
	drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv)
    {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
}

RadarController::~RadarController()
{
	printf("Disposing RPLIDAR driver.\n");
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
}

void RadarController::GetNodes(rplidar_response_measurement_node_hq_t (&nodes)[8192])
{
	size_t   count = _countof(nodes);

    op_result = drv->grabScanDataHq(nodes, count);
    
    if (IS_OK(op_result)) 
    {
        drv->ascendScanData(nodes, count);
	}
}

void RadarController::Initialize()
{
    ConnectionSuccess = false;
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
			ConnectionSuccess = true;
		}
		else
		{
			delete drv;
			drv = NULL;
		}
	}

}

bool RadarController::CheckRPLIDARHealth()
{
	rplidar_response_device_health_t healthinfo;
    u_result op_result = drv->getHealth(healthinfo);
    
    if (IS_OK(op_result)) 
    { 
		// the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR)
        {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            drv->reset();
            return false;
        }
       
        return true;
    }
    
    fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
    return false;
}

void RadarController::PrintInfo()
{
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
}

void RadarController::StartScan()
{
	printf("Starting RPLIDAR motor.\n");
    drv->startMotor();
	printf("Starting RPLIDAR scaning.\n");
    drv->startScan(0,1);
}

void RadarController::EndScan()
{
	printf("Stoping RPLIDAR motor.\n");
    drv->stop();
    drv->stopMotor();
}
