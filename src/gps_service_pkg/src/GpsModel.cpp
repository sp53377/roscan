///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#include "GpsModel.h"
#include "GpsService.h"
#include <GpsService/GpsMessages.h>
#include "GpsServiceTopics.h"
#include <CANFactory/CANTypes.h>
#include <iostream>
#include <chrono>
using namespace std::chrono_literals;

using namespace gps;

GpsModel::GpsModel()
{
}

ms64_t GpsModel::LastEpochTimestamp() const
{
   return Epoch.Timestamp;
}

void GpsModel::SetFlag(int flag)
{
   Flags |= flag;
}

void GpsModel::ClearFlags()
{
   Flags &= ~VEH_EPOCH_FLAGS;
   Flags &= ~IMPL_EPOCH_FLAGS;
}

void GpsModel::UpdateDevice()
{
   if(Device.Source < sc::NULL_ADDRESS && Device != KnownDevice)
   {
      KnownDevice = Device;
      //TODO static const hash_t gpsDeviceTopic = fps::ToHash(topics::gps_GpsDevice::Topic);
      //TODO PublishDevice(gpsDeviceTopic);
   }
}

void GpsModel::OnDisconnect()
{
}

void GpsModel::PublishDevice(const hash_t& topic)
{
	(void)topic;
   //TODO fps::Publish(topic, KnownDevice);
}

bool GpsModel::IsCompleteEpoch() const
{
   return ((Flags & VEH_EPOCH_FLAGS) == VEH_EPOCH_FLAGS) || ((Flags & IMPL_EPOCH_FLAGS) == IMPL_EPOCH_FLAGS);
}

void GpsModel::CheckTimestamps(EMessageFlag flag, const ms64_t& timestamp)
{
   ms64_t delta = timestamp - LastUpdate;
   if((Flags & (VEH_EPOCH_FLAGS | IMPL_EPOCH_FLAGS)) &&
      MESSAGE_WINDOW_MS < delta)
   {
      if(Device.Type == RT_VEHICLE)
      {
         std::cout << std::hex << "CheckTimestamps for 0x" << (int)Device.Source <<
         std::dec << " - failed with delta:" << delta <<
         std::hex << " flag: 0x" << flag <<
         " Flags: 0x" << Flags <<
         " All Flags: 0x" << AllFlags <<
         " src(0x" << (int)Epoch.Source << ")" <<
         std::dec << std::endl;
      }
      ClearFlags();
   }
   LastUpdate = timestamp;
}

void GpsModel::Update(EMessageFlag flag, const ms64_t& timestamp)
{
   AllFlags |= flag;
   CheckTimestamps(flag, timestamp);
   SetFlag(flag);

   if(IsCompleteEpoch())
   {
      UpdateDevice();
      Epoch.Direction = Direction.Get(timestamp);
      Epoch.Timestamp = timestamp;
      ServicePtr->PublishGpsEpoch(Epoch);
      ServicePtr->PublishGpsEx(Extended);
      PreviousTimestamp = timestamp;

      ClearFlags();
   }
}

void GpsModel::Handle(const VehiclePosition& msg, const ms64_t& timestamp)
{
   //If we got the extended info already
   if(Flags & MF_SatellitesUsed)
   {
      //Add the position to the extended info
      Epoch.Position.Lat += msg.Latitude;
      Epoch.Position.Lon += msg.Longitude;
   }
   else // We are the first (we'll add extended later)
   {
      Epoch.Position.Lat = msg.Latitude;
      Epoch.Position.Lon = msg.Longitude;
   }

   Update(MF_VehiclePosition, timestamp);
}

void GpsModel::Handle(const DifferentialStatus& msg, const ms64_t& timestamp)
{
   Epoch.DiffLocked = msg.DiffLocked;
   Extended.SignalToNoiseRatio = msg.SignalToNoiseRatio;
   Device.DiffSource = msg.DiffSource;
   Epoch.Accuracy = msg.Accuracy;
   Device.SFActive = msg.SFActive;
   Device.SF2Active = msg.SF2Active;
   Device.RTKActive = msg.RTKActive;
   Device.DualFrequency = msg.DualFrequency;
   Device.SFActivation = msg.SFActivation;
   Device.SF2License = msg.SF2License;
   Device.RTKActivation = msg.RTKActivation;
   Device.ATCapable = msg.ATCapable;

   Update(MF_DifferentialStatus, timestamp);
}

void GpsModel::Handle(const RollYawRate& msg, const ms64_t& timestamp)
{
   Epoch.CosRollAngle = msg.CosRollAngle;
   Epoch.RollAngle = msg.RollAngle;
   Epoch.YawRate = msg.YawRate;
   Extended.RTKDistance = msg.RTKDistance;
   Extended.AltitudeError = msg.AltitudeError;

   Update(MF_RollYawRate, timestamp);
}

void GpsModel::Handle(const PitchAltitude& msg, const ms64_t& timestamp)
{
   Epoch.Pitch = msg.Pitch;
   Epoch.Altitude = msg.Altitude;

   Update(MF_PitchAltitude, timestamp);
}

void GpsModel::Handle(const DirectionSpeed& msg, const ms64_t& timestamp)
{
   Epoch.Bearing = msg.Bearing;
   Epoch.Speed = msg.Speed;
   Epoch.Pitch = msg.Pitch;
   if(Flags & MF_SatellitesUsed)
   {
      Epoch.Altitude += msg.Altitude;
   }
   else
   {
      Epoch.Altitude = msg.Altitude;
   }

   Direction.UpdateCourseAndSpeed(Epoch.Bearing, Epoch.Speed, timestamp);
   Update(MF_DirectionSpeed, timestamp);
}

void GpsModel::Handle(const BearingSpeed& msg, const ms64_t& timestamp)
{
   Epoch.Bearing = msg.Bearing;
   Epoch.Speed = msg.Speed;

   Direction.UpdateCourseAndSpeed(Epoch.Bearing, Epoch.Speed, timestamp);
   Update(MF_BearingSpeed, timestamp);
}

void GpsModel::Handle(const TimeDate& msg, const ms64_t& timestamp)
{
   (void)msg;
   Update(MF_TimeDate, timestamp);
}

void GpsModel::Handle(const TerrainCompensation& msg, const ms64_t& timestamp)
{
   if(msg.IsValid)
   {
      if(msg.TCMode != AC_NotAvailable)
      {
         Device.TCMode = msg.TCMode;
      }
      Extended.TCAccuracy = msg.TCAccuracy;

      if(msg.LowSpeedCapable != AC_NotAvailable)
      {
         Extended.LowSpeedCapable = msg.LowSpeedCapable;
      }
      Extended.SolutionProjectionTime = msg.SolutionProjectionTime;
      Direction.UpdateNavBased(msg.Direction, Extended.LowSpeedCapable == AC_Active, timestamp);
   }
   else
   {
      Device.TCMode =  AC_Active;
      Epoch.Direction = D_Forward;
      Extended.LowSpeedCapable = AC_Active;
      Extended.SolutionProjectionTime = 0;
      Direction.UpdateNavBased(D_Forward, Extended.LowSpeedCapable == AC_Active, timestamp);
   }

   Update(MF_VehicleTerrainCompensation, timestamp);
}

void GpsModel::Handle(const GpsStatus& msg, const ms64_t& timestamp)
{
   Device.Mode = msg.Mode;
   Extended.State = msg.State;
   Extended.VelocitySolutionSatellites = msg.VelocitySolutionSatellites;
   Epoch.PDOP = msg.PDOP;
   Epoch.HDOP = msg.HDOP;
   Epoch.VDOP = msg.VDOP;
   Extended.CorrectionAge = msg.CorrectionAge;

   Update(MF_VehicleGpsStatus, timestamp);
}

void GpsModel::Handle(const SatellitesUsed& msg, const ms64_t& timestamp)
{
   Extended.SatellitesMask = msg.SatellitesMask;
   if(Flags & MF_VehiclePosition)  // If Vehicle Position came first
   {
      //Add the extended info
      Epoch.Position.Lat += msg.LatitudeEx;
      Epoch.Position.Lon += msg.LongitudeEx;
   }
   else
   {
      //Set to the extended info (We'll add to it later)
      Epoch.Position.Lat = msg.LatitudeEx;
      Epoch.Position.Lon = msg.LongitudeEx;
   }
   if(Flags & MF_DirectionSpeed)
   {
      Epoch.Altitude += msg.AltitudeEx;
   }
   else
   {
      Epoch.Altitude = msg.AltitudeEx;
   }

   Update(MF_SatellitesUsed, timestamp);
}

void GpsModel::SetCANDevice(const sc::CANDevice& device)
{
   Device.Name = device.Name;
   Device.Source = device.Address;
   Device.Node = device.Node;
   Epoch.Source = device.Address;
   Epoch.Node = device.Node;
   Extended.Source = device.Address;
   Device.Type = RT_UNKNOWN;
}

void GpsModel::Handle(const ReceiverInfo& msg, const ms64_t& timestamp)
{
   Device.LicenseDays = msg.LicenseDays;
   Device.HardwareVersion = msg.HardwareVersion;
   Device.SoftwareVersion = msg.SoftwareVersion;
   Device.SerialNumber = msg.SerialNumber;
   Epoch.SerialNumber = msg.SerialNumber;
   Extended.SerialNumber = msg.SerialNumber;
   Device.ReceiverVersion = msg.ReceiverVersion;
   Update(MF_ReceiverInfo, timestamp);
}

void GpsModel::Handle(const HeadingMode& msg, const ms64_t& timestamp)
{
   Extended.NavigationMode = msg.NavigationMode;
   Extended.NavigationLocked = msg.NavigationLocked;
   Extended.HorizontalErrorEstimate = msg.HorizontalErrorEstimate + (msg.HorizontalErrorEstimateExtendedPrecision / 10);
   Extended.ActivationDelta = msg.ActivationDelta;
   Extended.Sequence = msg.Sequence;
   Extended.Decode = msg.Decode;
   Extended.SolutionStatus = msg.SolutionStatus;
   Update(MF_HeadingMode, timestamp);
}

void GpsModel::Handle(const MachineSelectedSpeed& msg, const ms64_t& timestamp)
{
   Direction.UpdateTransmissionBasedDirection(msg.Direction, timestamp);
}

void GpsModel::Handle(const WheelBasedSpeedAndDistance& msg, const ms64_t& timestamp)
{
   Direction.UpdateWheelBasedDirection(msg.MachineDirection, timestamp);
}

sc::addr_t GpsModel::GetAddress() const
{
   return Device.Source;
}

void GpsModel::SetGpsService(GpsService* service)
{
   ServicePtr = service;
}