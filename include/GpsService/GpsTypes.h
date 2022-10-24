///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <GpsService/GpsMessages.h>

namespace gps
{
   static constexpr const int64_t GPS_UPDATE_RATE = 200;
   static constexpr const int64_t SYNC_WINDOW = GPS_UPDATE_RATE * 3 / 8;//Time window that all epochs must reside in (75 ms)
   static constexpr const int64_t EPOCH_TIMEOUT = GPS_UPDATE_RATE + SYNC_WINDOW;

   static constexpr const uint32_t INVALID_SERIAL_NUMBER = 0xFFFFFFFF;
   struct GpsEpoch
   {
      world_t Position;
      uint8_t Accuracy = 0;
      float CosRollAngle = 0.0f;
      deg32_t RollAngle = 0.0f;
      dps32_t YawRate = 0.0f;
      deg32_t Pitch = 0.0f;
      m32_t Altitude = 0.0f;
      kph32_t Speed = 0.0f;
      deg32_t Bearing = 0.0f;
      EDirection Direction = D_NotAvailable;
      ELocked DiffLocked = LO_NotAvaialble;
      uint8_t PDOP = 0;
      uint8_t HDOP = 0;
      uint8_t VDOP = 0;
      uint32_t SerialNumber = INVALID_SERIAL_NUMBER;
      int64_t Timestamp = 0ll;
      sc::addr_t Source = sc::NULL_ADDRESS;
      sc::node_t Node = sc::INVALID_NODE;
   };

   struct GpsDevice
   {
      EDifferentialSource DiffSource = DS_NotAvailable;
      EActive SFActive = AC_NotAvailable;
      EActive SF2Active = AC_NotAvailable;
      EActive RTKActive = AC_NotAvailable;
      EActive DualFrequency = AC_NotAvailable;
      EActive SFActivation = AC_NotAvailable;
      EActive SF2License = AC_NotAvailable;
      EActive RTKActivation = AC_NotAvailable;
      EATCapable ATCapable = AT_NotAvailable;
      EActive TCMode = AC_NotAvailable;
      EPositionMode Mode = PM_Undefined;
      uint16_t LicenseDays = 0;
      uint8_t HardwareVersion = 0;
      uint8_t SoftwareVersion = 0;
      uint32_t SerialNumber = INVALID_SERIAL_NUMBER;
      EReceiverVersion ReceiverVersion = RV_NotAvailable;
      sc::IsoName Name;
      sc::addr_t Source = sc::NULL_ADDRESS;
      EReceiverType Type = RT_UNKNOWN;
      sc::node_t Node = sc::INVALID_NODE;

      void Reset()
      {
         DiffSource = DS_NotAvailable;
         SFActive = AC_NotAvailable;
         SF2Active = AC_NotAvailable;
         RTKActive = AC_NotAvailable;
         DualFrequency = AC_NotAvailable;
         SFActivation = AC_NotAvailable;
         SF2License = AC_NotAvailable;
         RTKActivation = AC_NotAvailable;
         ATCapable = AT_NotAvailable;
         TCMode = AC_NotAvailable;
         Mode = PM_Undefined;
         LicenseDays = 0;
         HardwareVersion = 0;
         SoftwareVersion = 0;
         SerialNumber = 0;
         ReceiverVersion = RV_NotAvailable;
         Source = sc::NULL_ADDRESS;
         Name.Reset();
      }

      bool operator!=(const GpsDevice& rhs) const
      {
         return !((rhs.DiffSource == DiffSource) &&
                 (rhs.SFActive == SFActive) &&
                 (rhs.SF2Active == SF2Active) &&
                 (rhs.RTKActive == RTKActive) &&
                 (rhs.DualFrequency == DualFrequency) &&
                 (rhs.SFActivation == SFActivation) &&
                 (rhs.SF2License == SF2License) &&
                 (rhs.RTKActivation == RTKActivation) &&
                 (rhs.ATCapable == ATCapable) &&
                 (rhs.TCMode == TCMode) &&
                 (rhs.Mode == Mode) &&
                 (rhs.LicenseDays == LicenseDays) &&
                 (rhs.HardwareVersion == HardwareVersion) &&
                 (rhs.SoftwareVersion == SoftwareVersion) &&
                 (rhs.SerialNumber == SerialNumber) &&
                 (rhs.ReceiverVersion == ReceiverVersion) &&
                 (rhs.Name == Name) &&
                 (rhs.Source == Source));
      }

   };

   struct GpsEx
   {
      ELocked NavigationLocked = LO_NotAvaialble;
      sec32_t ActivationDelta = 0;
      EDifferentialSource NavigationMode = DS_NotAvailable;
      uint8_t VelocitySolutionSatellites = 0;
      ECorrectionState State = CS_NotAvailable;
      uint32_t SatellitesMask = 0;
      uint8_t TCAccuracy = 0;
      db32_t SignalToNoiseRatio = 0.0f;
      km32_t RTKDistance = 0.0f;
      mm32_t AltitudeError = 0.0f;
      deg32_t Pitch = 0.0f;
      m32_t Altitude = 0.0f;
      deg32_t Bearing = 0.0f;
      kph32_t Speed = 0.0f;
      ms32_t SolutionProjectionTime = 0;
      sec32_t CorrectionAge = 0;
      cm32_t HorizontalErrorEstimate = 0.0f;
      ECorrect Sequence = CO_NotAvaialble;
      ECorrect Decode = CO_NotAvaialble;
      ESolutionStatus SolutionStatus = SST_NotAvailable;
      EActive LowSpeedCapable = AC_NotAvailable;
      sc::addr_t Source = sc::NULL_ADDRESS;
      uint32_t SerialNumber = INVALID_SERIAL_NUMBER;
   };
}
