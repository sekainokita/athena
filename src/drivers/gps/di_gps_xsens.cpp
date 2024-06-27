
/******************************************************************************
*
* Copyright (C) 2023 - 2028 KETI, All rights reserved.
*                           (Korea Electronics Technology Institute)
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running for Korean Government Project, or
* (b) that interact with KETI project/platform.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* KETI BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the KETI shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from KETI.
*
******************************************************************************/
/******************************************************************************/
/**
*
* @file di_gps_xsens.c
*
* This file contains a DI GPS for XSENS device
*
* @note
*
* V2X DI GPS XSENS Source File
*
* MODIFICATION HISTORY:
* Ver   Who  Date     Changes
* ----- ---- -------- ----------------------------------------------------
* 1.00  bman  23.07.10 First release
*
******************************************************************************/

/***************************** Include ***************************************/
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>

#include <iostream>
#include <iomanip>
#include <list>
#include <string>
#include "di_gps_xsens.h"
#include "type.h"

#if defined(CONFIG_NVIDIA_JETSON_UTILS)
#include "videoSource.h"
#include "videoOutput.h"

#include "logging.h"
#include "commandLine.h"
#endif

/***************************** Definition ************************************/
//#define DI_GPS_XSENS_DEBUG  (1)

using namespace std;

/***************************** Enum and Structure ****************************/
class CallbackHandler : public XsCallback
{
public:
	CallbackHandler(size_t maxBufferSize = 5)
		: m_maxNumberOfPacketsInBuffer(maxBufferSize)
		, m_numberOfPacketsInBuffer(0)
	{
	}

	virtual ~CallbackHandler() throw()
	{
	}

	bool packetAvailable() const
	{
		xsens::Lock locky(&m_mutex);
		return m_numberOfPacketsInBuffer > 0;
	}

	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		xsens::Lock locky(&m_mutex);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override
	{
		xsens::Lock locky(&m_mutex);
		assert(packet != 0);
		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
			(void)getNextPacket();

		m_packetBuffer.push_back(*packet);
		++m_numberOfPacketsInBuffer;
		assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
	}
private:
	mutable xsens::Mutex m_mutex;

	size_t m_maxNumberOfPacketsInBuffer;
	size_t m_numberOfPacketsInBuffer;
	list<XsDataPacket> m_packetBuffer;
};

/***************************** Static Variable *******************************/
Journaller* gJournal = NULL;
static XsControl* s_CXsControl = NULL;
static XsDevice* s_pstXsDevice = NULL;
static XsPortInfo s_mtPort;
static CallbackHandler sh_CCallBack;
//#define DI_GPS_XSENS_DEBUG      (1)

/***************************** Function  *************************************/

void P_DI_GPS_XSENS_GetAging(void)
{
	PrintTrace("\nMain loop. Recording data for 10 seconds.");

	int64_t startTime = XsTime::timeStampNow();
	while (XsTime::timeStampNow() - startTime <= 10000)
	{
		if (sh_CCallBack.packetAvailable())
		{
			cout << setw(5) << fixed << setprecision(5);

			XsDataPacket packet = sh_CCallBack.getNextPacket();
			if (packet.containsCalibratedData())
			{
				XsVector acc = packet.calibratedAcceleration();
				cout << "\r"
					<< "Acc X:" << acc[0]
					<< ", Acc Y:" << acc[1]
					<< ", Acc Z:" << acc[2];
				XsVector gyr = packet.calibratedGyroscopeData();
				cout << " |Gyr X:" << gyr[0]
					<< ", Gyr Y:" << gyr[1]
					<< ", Gyr Z:" << gyr[2];
				XsVector mag = packet.calibratedMagneticField();
				cout << " |Mag X:" << mag[0]
					<< ", Mag Y:" << mag[1]
					<< ", Mag Z:" << mag[2];
			}

			if (packet.containsOrientation())
			{
				XsQuaternion quaternion = packet.orientationQuaternion();
				cout << "\r"
					<< "q0:" << quaternion.w()
					<< ", q1:" << quaternion.x()
					<< ", q2:" << quaternion.y()
					<< ", q3:" << quaternion.z();

				XsEuler euler = packet.orientationEuler();
				cout << " |Roll:" << euler.roll()
					<< ", Pitch:" << euler.pitch()
					<< ", Yaw:" << euler.yaw();
			}

			if (packet.containsLatitudeLongitude())
			{
				XsVector latLon = packet.latitudeLongitude();
				cout << " |Lat:" << latLon[0]
					<< ", Lon:" << latLon[1];
			}

			if (packet.containsAltitude())
            {
				cout << " |Alt:" << packet.altitude();
            }

			if (packet.containsVelocity())
			{
				XsVector vel = packet.velocity(XDI_CoordSysEnu);
				cout << " |E:" << vel[0]
					<< ", N:" << vel[1]
					<< ", U:" << vel[2];
			}

			cout << flush;
		}
		XsTime::msleep(0);
	}
}

int32_t DI_GPS_XSENS_Get(DI_GPS_XSENS_T *pstDiGpsXsens)
{
    int32_t nRet = DI_ERROR;

    if(pstDiGpsXsens == NULL)
    {
        PrintError("pstDiGpsXsens == NULL!!");
        return nRet;
    }

#if defined(DI_GPS_XSENS_DEBUG)
    (void)P_DI_GPS_XSENS_GetAging();
#else
    if (sh_CCallBack.packetAvailable() == TRUE)
    {
        XsDataPacket packet = sh_CCallBack.getNextPacket();
        if (packet.containsCalibratedData())
        {
            XsVector acc = packet.calibratedAcceleration();
#if defined(DI_GPS_XSENS_DEBUG)
            cout << "\r" << "Acc X:" << acc[0] << ", Acc Y:" << acc[1] << ", Acc Z:" << acc[2];
#endif
            pstDiGpsXsens->fAccX = acc[0];
            pstDiGpsXsens->fAccY = acc[1];
            pstDiGpsXsens->fAccZ = acc[2];

            XsVector gyr = packet.calibratedGyroscopeData();
#if defined(DI_GPS_XSENS_DEBUG)
            cout << " |Gyr X:" << gyr[0] << ", Gyr Y:" << gyr[1] << ", Gyr Z:" << gyr[2];
#endif
            pstDiGpsXsens->fGyrX = gyr[0];
            pstDiGpsXsens->fGyrY = gyr[1];
            pstDiGpsXsens->fGyrZ = gyr[2];

            XsVector mag = packet.calibratedMagneticField();
#if defined(DI_GPS_XSENS_DEBUG)
            cout << " |Mag X:" << mag[0] << ", Mag Y:" << mag[1] << ", Mag Z:" << mag[2];
#endif
            pstDiGpsXsens->fMagX = mag[0];
            pstDiGpsXsens->fMagY = mag[1];
            pstDiGpsXsens->fMagZ = mag[2];
        }

        if (packet.containsOrientation())
        {
            XsQuaternion quaternion = packet.orientationQuaternion();
#if defined(DI_GPS_XSENS_DEBUG)
            cout << "\r" << "q0:" << quaternion.w() << ", q1:" << quaternion.x() << ", q2:" << quaternion.y() << ", q3:" << quaternion.z();
#endif
            pstDiGpsXsens->fQuaternionW = quaternion.w();
            pstDiGpsXsens->fQuaternionX = quaternion.x();
            pstDiGpsXsens->fQuaternionY = quaternion.y();
            pstDiGpsXsens->fQuaternionZ = quaternion.z();

            XsEuler euler = packet.orientationEuler();
#if defined(DI_GPS_XSENS_DEBUG)
            cout << " |Roll:" << euler.roll() << ", Pitch:" << euler.pitch() << ", Yaw:" << euler.yaw();
#endif
            pstDiGpsXsens->fEulerRoll = euler.roll();
            pstDiGpsXsens->fEulerPitch = euler.pitch();
            pstDiGpsXsens->fEulerYaw = euler.yaw();
        }

        if (packet.containsLatitudeLongitude())
        {
            XsVector latLon = packet.latitudeLongitude();
#if defined(DI_GPS_XSENS_DEBUG)
            cout << " |Lat:" << latLon[0] << ", Lon:" << latLon[1];
#endif
            pstDiGpsXsens->fLatitude = latLon[0];
            pstDiGpsXsens->fLongitude = latLon[1];
        }

        if (packet.containsAltitude())
        {
#if defined(DI_GPS_XSENS_DEBUG)
            cout << " |Alt:" << packet.altitude();
#endif
            pstDiGpsXsens->fAltitude = packet.altitude();
        }


        if (packet.containsVelocity())
        {
            XsVector vel = packet.velocity(XDI_CoordSysEnu);
#if defined(DI_GPS_XSENS_DEBUG)
            cout << " |E:" << vel[0] << ", N:" << vel[1] << ", U:" << vel[2];
#endif
            pstDiGpsXsens->fVelocityEast = vel[0];
            pstDiGpsXsens->fVelocityNorth = vel[1];
            pstDiGpsXsens->fVelocityUp = vel[2];
        }

        nRet = DI_OK;
    }
    else
    {
        PrintError("sh_CCallBack.packetAvailable() is false!");
    }
#endif

    return nRet;
}

int32_t DI_GPS_XSENS_Open(DI_GPS_XSENS_T *pstDiGpsXsens)
{
    int32_t nRet = DI_ERROR;

    if(pstDiGpsXsens == NULL)
    {
        PrintError("pstDiGpsXsens == NULL!!");
        return nRet;
    }

	PrintDebug("Scanning for devices...");
	XsPortInfoArray portInfoArray = XsScanner::scanPorts();
	for (auto const &portInfo : portInfoArray)
	{
		if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
		{
			s_mtPort = portInfo;
			break;
		}
	}

	if (s_mtPort.empty())
    {
        PrintError("s_mtPort.empty() is failed! No MTi device found!");
        return nRet;
    }

	cout << "Found a device with ID: " << s_mtPort.deviceId().toString().toStdString() << " @ port: " << s_mtPort.portName().toStdString() << ", baudrate: " << s_mtPort.baudrate() << endl;

	PrintDebug("Opening port...");
	if (!s_CXsControl->openPort(s_mtPort.portName().toStdString(), s_mtPort.baudrate()))
    {
        PrintError("s_CXsControl->openPort() is failed! Could not open port!");
        return nRet;

    }

	// Get the device object
	s_pstXsDevice = s_CXsControl->device(s_mtPort.deviceId());
	assert(s_pstXsDevice != 0);

	cout << "Device: " << s_pstXsDevice->productCode().toStdString() << ", with ID: " << s_pstXsDevice->deviceId().toString() << " opened." << endl;

	// Create and attach callback handler to device
	s_pstXsDevice->addCallbackHandler(&sh_CCallBack);

	// Put the device into configuration mode before configuring the device
	PrintDebug("Putting device into configuration mode...");
	if (!s_pstXsDevice->gotoConfig())
    {
        PrintError("device->gotoConfig() is failed! Could not put device into configuration mode!");
        return nRet;
    }

	PrintDebug("Configuring the device...");

	// Important for Public XDA!
	// Call this function if you want to record a mtb file:
	s_pstXsDevice->readEmtsAndDeviceConfiguration();

	XsOutputConfigurationArray configArray;
	configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));

	if (s_pstXsDevice->deviceId().isImu())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 100));
		configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));
		configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
	}
	else if (s_pstXsDevice->deviceId().isVru() || s_pstXsDevice->deviceId().isAhrs())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
	}
	else if (s_pstXsDevice->deviceId().isGnss())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
		configArray.push_back(XsOutputConfiguration(XDI_LatLon, 100));
		configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid, 100));
		configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ, 100));
	}
	else
	{
        PrintError("Unknown device while configuring!");
        return nRet;
	}

	if (!s_pstXsDevice->setOutputConfiguration(configArray))
    {
        PrintError("device->setOutputConfiguration() is failed! Could not configure MTi device!");
        return nRet;
    }

	PrintDebug("Putting device into measurement mode...");
	if (!s_pstXsDevice->gotoMeasurement())
    {
        PrintError("device->gotoMeasurement() is failed! Could not put device into measurement mode!");
        return nRet;
    }

    nRet = DI_OK;

    return nRet;
}

int32_t DI_GPS_XSENS_Close(DI_GPS_XSENS_T *pstDiGpsXsens)
{
    int32_t nRet = DI_ERROR;

    if(pstDiGpsXsens == NULL)
    {
        PrintError("pstDiGpsXsens == NULL!!");
        return nRet;
    }

	PrintDebug("Closing port...");
	s_CXsControl->closePort(s_mtPort.portName().toStdString());

    nRet = DI_OK;

    return nRet;
}

int32_t DI_GPS_XSENS_Init(DI_GPS_XSENS_T *pstDiGpsXsens)
{
    int32_t nRet = DI_ERROR;

    if(pstDiGpsXsens == NULL)
    {
        PrintError("pstDiGpsXsens == NULL!!");
        return nRet;
    }

    s_CXsControl = XsControl::construct();
    if(s_CXsControl == NULL)
    {
        PrintError("XsControl::construct() is failed! s_CXsControl is NULL!");
        return nRet;
    }
    else
    {
        PrintDebug("Successfully created XsControl object");
        nRet = DI_OK;
    }

	return nRet;
}

int32_t DI_GPS_XSENS_DeInit(DI_GPS_XSENS_T *pstDiGpsXsens)
{
    int32_t nRet = DI_ERROR;

    if(pstDiGpsXsens == NULL)
    {
        PrintError("pstDiGpsXsens == NULL!!");
        return nRet;
    }

	s_CXsControl->destruct();
    s_CXsControl = NULL;

    PrintDebug("Successfully destroyed XsControl object");

    nRet = DI_OK;

	return nRet;
}

