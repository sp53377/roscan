///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <GpsService/GpsTypes.h>
#include "DirectionCoordinator.h"

namespace gps
{
   struct VehiclePosition;
   struct DifferentialStatus;
   struct RollYawRate;
   struct PitchAltitude;
   struct DirectionSpeed;
   struct BearingSpeed;
   struct TimeDate;
   struct TerrainCompensation;
   struct GpsStatus;
   struct SatellitesUsed;
   struct ReceiverInfo;
   struct HeadingMode;
   struct WheelBasedSpeedAndDistance;
   struct MachineSelectedSpeed;

   class GpsService;

   class GpsModel
   {
   public:
      GpsModel();

      void SetCANDevice(const sc::CANDevice& device);
      void PublishDevice(const hash_t& topic);
      void OnDisconnect();
      ms64_t LastEpochTimestamp() const;
      sc::addr_t GetAddress() const;
      void SetGpsService(GpsService* service);

      void Handle(const VehiclePosition& msg, const ms64_t& timestamp);
      void Handle(const DifferentialStatus& msg, const ms64_t& timestamp);
      void Handle(const RollYawRate& msg, const ms64_t& timestamp);
      void Handle(const PitchAltitude& msg, const ms64_t& timestamp);
      void Handle(const DirectionSpeed& msg, const ms64_t& timestamp);
      void Handle(const BearingSpeed& msg, const ms64_t& timestamp);
      void Handle(const TimeDate& msg, const ms64_t& timestamp);
      void Handle(const TerrainCompensation& msg, const ms64_t& timestamp);
      void Handle(const GpsStatus& msg, const ms64_t& timestamp);
      void Handle(const SatellitesUsed& msg, const ms64_t& timestamp);
      void Handle(const ReceiverInfo& msg, const ms64_t& timestamp);
      void Handle(const HeadingMode& msg, const ms64_t& timestamp);
      void Handle(const MachineSelectedSpeed& msg, const ms64_t& timestamp);
      void Handle(const WheelBasedSpeedAndDistance&, const ms64_t& timestamp);

   private:
      enum
      {
         MESSAGE_WINDOW_MS = 100
      };

      enum MessageIndex
      {
         BIT_VehiclePosition,    // 0
         BIT_DifferentialStatus, // 1
         BIT_RollYawRate,        // 2
         BIT_PitchAltitude,      // 3
         BIT_DirectionSpeed,     // 4
         BIT_BearingSpeed,       // 5
         BIT_TimeDate,           // 6
         BIT_TerrainCompensation,// 7
         BIT_GpsStatus,          // 8
         BIT_SatellitesUsed,     // 9
         BIT_ReceiverInfo,       // 10
         BIT_HeadingMode,        // 11
         BIT_NumberOfMessages    // 12
      };

      enum EMessageFlag
      {
         MF_VehiclePosition = (1 << BIT_VehiclePosition),
         MF_DifferentialStatus = (1 << BIT_DifferentialStatus),
         MF_RollYawRate = (1 << BIT_RollYawRate),
         MF_PitchAltitude = (1 << BIT_PitchAltitude),
         MF_DirectionSpeed = (1 << BIT_DirectionSpeed),
         MF_BearingSpeed = (1 << BIT_BearingSpeed),
         MF_TimeDate = (1 << BIT_TimeDate),
         MF_VehicleTerrainCompensation = (1 << BIT_TerrainCompensation),
         MF_VehicleGpsStatus = (1 << BIT_GpsStatus),
         MF_SatellitesUsed = (1 << BIT_SatellitesUsed),
         MF_ReceiverInfo = (1 << BIT_ReceiverInfo),
         MF_HeadingMode = (1 << BIT_HeadingMode)
      };

      enum
      {
         VEH_EPOCH_FLAGS =
            MF_VehiclePosition |
            MF_DifferentialStatus |
            MF_RollYawRate |
            MF_DirectionSpeed |
            MF_SatellitesUsed,

         IMPL_EPOCH_FLAGS =
            MF_VehiclePosition |
            MF_DifferentialStatus |
            MF_RollYawRate |
            MF_BearingSpeed |
            MF_SatellitesUsed

            //TODO Where do we get Heading From (there are multiple sources)
      };

      void SetFlag(int flag);
      void ClearFlags();
      void CheckTimestamps(EMessageFlag flag, const ms64_t& timestamp);
      void Update(EMessageFlag flag, const ms64_t& timestamp);
      void UpdateDevice();
      bool IsCompleteEpoch() const;
      void CheckConnect();

      GpsService* ServicePtr=nullptr;
      ms64_t LastUpdate = 0ll;
      uint32_t Flags = 0;
      uint32_t AllFlags = 0;
      GpsEpoch Epoch;
      GpsDevice Device;
      GpsDevice KnownDevice;
      GpsEx Extended;
      DirectionCoordinator Direction;
      ms64_t PreviousTimestamp { 0 };
   };
}
