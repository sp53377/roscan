///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <CommonCore/CommonTypes.h>
#include <JDMath/ParameterScaling.h>
#include <CANFactory/CANTypes.h>

namespace gps
{
   constexpr uint8_t RESERVED_BYTE = 0xFF;
   constexpr uint32_t RESERVED_WORD = 0xFFFFFFFF;

   enum EATCapable
   {
      AT_Capable = 0x4154,//"AT"
      AT_Active = 0x4750,//"GP" Grace Period
      AT_NotAvailable = 0x0000
   };

   enum EDifferentialSource
   {
      DS_None = 0,
      DS_Other = 1,
      DS_JD = 2,
      DS_WAAS = 3,
      DS_SF1 = 4,
      DS_SF2 = 5,
      DS_RTKX = 6,
      DS_RTK = 7,
      DS_Reserved0 = 8,
      DS_Reserved1 = 9,
      DS_Reserved2 = 10,
      DS_Reserved3 = 11,
      DS_Reserved4 = 12,
      DS_Reserved5 = 13,
      DX_Error = 14,
      DS_NotAvailable = 15
   };

   enum EPositionMode
   {
      PM_Undefined = 0,
      PM_NoFix = 1,
      PM_2D = 2,
      PM_3D = 3
   };

   enum ECorrectionState
   {
      CS_NoFix = 0,
      CS_FixNoGNSS = 1,
      CS_FixWithDifferential = 2,
      CS_Reserved1 = 3,
      CS_Reserved2 = 4,
      CS_Reserved3 = 5,
      CS_Reserved4 = 6,
      CS_Reserved5 = 7,
      CS_Reserved6 = 8,
      CS_Reserved7 = 9,
      CS_Reserved8 = 10,
      CS_Reserved9 = 11,
      CS_Reserved10 = 12,
      CS_Reserved11 = 13,
      CS_Error = 14,
      CS_NotAvailable = 15
   };

   enum EReceiverVersion
   {
      RV_Gen2 = 0,
      RV_Gen4 = 1,
      RV_LowCost = 2,
      RV_Gen5 = 3,
      RV_Gen6 = 4,
      RV_SF7000 = 5,
      RV_Reserved2 = 6,
      RV_Reserved3 = 7,
      RV_Reserved4 = 8,
      RV_Reserved5 = 9,
      RV_Reserved6 = 10,
      RV_Reserved7 = 11,
      RV_Reserved8 = 12,
      RV_Reserved9 = 13,
      RV_Error = 14,
      RV_NotAvailable = 15
   };

   enum ESolutionStatus
   {
      SST_OutOfRange = 0,
      SST_NoFailure = 1,
      SST_NotEnoughGPSMeasurementsToInitialize = 2,
      SST_NoInitialPosition =3,
      SST_InProgress =4,
      SST_NotEnoughGPSMeasurementsToContinude =5,
      SST_LowPDOP =6,
      SST_NoVelocitySolution =7,
      SST_UpdateTooLarge = 8,
      SST_HeightVelocityLimitsExceeded =9,
      SST_DisabledOrNotAuthorized = 10,
      SST_MissingPVT = 11,
      SST_Reserved1 = 12,
      SST_Reserved2 = 13,
      SST_Error =14,
      SST_NotAvailable = 15
   };

   ///////////////////////////////////////////
   //Messages
   ///////////////////////////////////////////
   struct VehiclePosition
   {
      lat64_t Latitude = 0.0;
      lon64_t Longitude = 0.0;
   };

   struct CANVehiclePosition
   {
      enum { PGN = 0xFEF3 };
      enum { PRIORITY = 6 };
      uint32_t Latitude : 32;
      uint32_t Longitude : 32;

      static inline VehiclePosition FromCAN(const sc::CANMsg& msg)
      {
	 const auto& in = sc::ToMsg<CANVehiclePosition>(msg);
         VehiclePosition out;
         out.Latitude = cc::ScaledNoNaN64(in.Latitude, 0.0000001, 210.0);
         out.Longitude = cc::ScaledNoNaN64(in.Longitude, 0.0000001, 210.0);
         return out;
      }

      static inline CANVehiclePosition ToCAN(const VehiclePosition& in)
      {
         CANVehiclePosition msg;
         msg.Latitude = cc::UnscaledNoNaN64<uint32_t>(in.Latitude, 0.0000001, 210.0);
         msg.Longitude = cc::UnscaledNoNaN64<uint32_t>(in.Longitude, 0.0000001, 210.0);
         return msg;
      }

   };
   ASSERT_EIGHT(CANVehiclePosition);

   ///////////////////////////////////////////
   enum EAugmentationType
   {
	AUT_WAAS,
	AUT_EGNOS,
	AUT_MSAS,
	AUT_GAGAN,
	AUT_SNAS,
	AUT_WADGPS,
	AUT_SDCM,
	AUT_RESERVED1,
	AUT_RESERVED2,
	AUT_RESERVED3,
	AUT_RESERVED4,
	AUT_RESERVED5,
	AUT_RESERVED6,
	AUT_RESERVED7,
	AUT_Error,
	AUT_NotAvailable
   };

   struct DifferentialStatus
   {
      ELocked DiffLocked { LO_NotAvaialble };
      EAugmentationType AugmentationType { AUT_NotAvailable };
      db32_t SignalToNoiseRatio;//0.1 dB/bit
      EDifferentialSource DiffSource { DS_NotAvailable };
      uint8_t Accuracy { 0 };
      EActive SFActive { AC_NotAvailable };
      EActive SF2Active { AC_NotAvailable };
      EActive RTKActive { AC_NotAvailable };
      EActive DualFrequency { AC_NotAvailable };
      EActive SFActivation { AC_NotAvailable };
      EActive SF2License { AC_NotAvailable };
      EActive RTKActivation { AC_NotAvailable };
      EATCapable ATCapable { AT_NotAvailable };
   };

   struct CANDifferentialStatus
   {
      enum { PGN = 0xFFFF };
      enum { CMD1 = 0x53 };
      enum {PRIORITY = 3};

      uint8_t Cmd : 8;
      uint8_t DiffLocked : 2;
      uint8_t Reserved1 : 2;
      uint8_t AugmentationType : 4;

      uint8_t SignalToNoiseRatio : 8;//0.1 dB/bit

      uint8_t DiffSource : 4;
      uint8_t Accuracy : 4;
      
      uint8_t SFActive : 2;
      uint8_t SF2Active : 2;
      uint8_t RTKActive : 2;
      uint8_t DualFrequency : 2;

      uint8_t SFActivation : 2;
      uint8_t SF2License : 2;
      uint8_t RTKActivation : 2;

      uint16_t ATCapable : 16;

      static inline DifferentialStatus FromCAN(const sc::CANMsg& msg)
      {
	 const auto& in = sc::ToMsg<CANDifferentialStatus>(msg);
         DifferentialStatus out;
         out.DiffLocked = static_cast<ELocked>(in.DiffLocked);
	 out.AugmentationType = static_cast<EAugmentationType>(in.AugmentationType);
         out.SignalToNoiseRatio = cc::ScaledNoNaN(in.SignalToNoiseRatio, 0.1f);
         out.DiffSource = static_cast<EDifferentialSource>(in.DiffSource);
         out.Accuracy = in.Accuracy;
         out.SFActive = static_cast<EActive>(in.SFActive);
         out.SF2Active = static_cast<EActive>(in.SF2Active);
         out.RTKActive = static_cast<EActive>(in.RTKActive);
         out.DualFrequency = static_cast<EActive>(in.DualFrequency);
         out.SFActivation = static_cast<EActive>(in.SFActivation);
         out.SF2License = static_cast<EActive>(in.SF2License);
         out.RTKActivation = static_cast<EActive>(in.RTKActivation);
         out.ATCapable = static_cast<EATCapable>(in.ATCapable);
         return out;
      }

      static inline CANDifferentialStatus ToCAN(const DifferentialStatus& in)
      {
	CANDifferentialStatus out;
	out.Cmd = static_cast<uint8_t>(CANDifferentialStatus::CMD1);
        out.DiffLocked = static_cast<uint8_t>(in.DiffLocked);
	out.AugmentationType = static_cast<uint8_t>(in.AugmentationType);
        out.SignalToNoiseRatio = cc::Unscaled<uint8_t>(in.SignalToNoiseRatio, 0.1f);
        out.DiffSource = static_cast<uint8_t>(in.DiffSource);
        out.Accuracy = in.Accuracy;
        out.SFActive = static_cast<int8_t>(in.SFActive);
        out.SF2Active = static_cast<uint8_t>(in.SF2Active);
        out.RTKActive = static_cast<uint8_t>(in.RTKActive);
        out.DualFrequency = static_cast<uint8_t>(in.DualFrequency);
	out.SFActivation = static_cast<uint8_t>(in.SFActivation);
        out.SF2License = static_cast<uint8_t>(in.SF2License);
        out.RTKActivation = static_cast<uint8_t>(in.RTKActivation);
        out.ATCapable = static_cast<uint16_t>(in.ATCapable);
        return out;
      }

   };
   ASSERT_EIGHT(CANDifferentialStatus);

   ///////////////////////////////////////////
   struct RollYawRate
   {
      float CosRollAngle;
      deg32_t RollAngle;
      dps32_t YawRate;
      km32_t RTKDistance;
      mm32_t AltitudeError;
   };

   struct CANRollYawRate
   {
      enum { PGN = 0xFFFF };
      enum { PRIORITY = 3 };

      uint8_t Cmd : 8;
      uint8_t CosRollAngle : 8;
      uint16_t RollAngle : 16;
      uint16_t YawRate : 16;
      uint8_t RTKDistance : 8;
      uint8_t AltitudeError : 8;

      static inline RollYawRate FromCAN(const sc::CANMsg& msg)
      {
	 const auto& in = sc::ToMsg<CANRollYawRate>(msg);
         RollYawRate out;
         out.CosRollAngle = cc::ScaledNoNaN(in.CosRollAngle, 1.0f / 1024.0f, -800.0f / 1024.0f);
         out.RollAngle = cc::ScaledNoNaN(in.RollAngle, 1.0f / 128.0f, 200.0f);
         out.YawRate = cc::ScaledNoNaN(in.YawRate, 1.0f / 128.0f, 200.0f);
         out.RTKDistance = cc::ScaledNoNaN(in.RTKDistance, 0.2f);
         out.AltitudeError = cc::ScaledNoNaN(in.AltitudeError, 1.0f);
	 return out;
      }

   protected:
      template<typename T> static T ToCAN(const RollYawRate& in)
      {
         T msg;
         msg.Cmd = 0xFF;
         msg.CosRollAngle = cc::Unscaled<uint8_t>(in.CosRollAngle, 1.0f / 1024.0f, -800.0f / 1024.0f);
         msg.RollAngle = cc::Unscaled<uint16_t>(in.RollAngle, 1.0f / 128.0f, 200.0f);
         msg.YawRate = cc::Unscaled<uint16_t>(in.YawRate, 1.0f / 128.0f, 200.0f);
         msg.RTKDistance = cc::Unscaled<uint8_t>(in.RTKDistance, 0.2f);
         msg.AltitudeError = cc::Unscaled<uint8_t>(in.AltitudeError, 1.0f);
         return msg;
      }

   };
   ASSERT_EIGHT(CANRollYawRate);

   struct CANRollYawRateVehicle: public CANRollYawRate
   {
      enum { CMD1 = 0xE1 };

      static inline CANRollYawRateVehicle ToCAN(const RollYawRate& in)
      {
         CANRollYawRateVehicle msg = CANRollYawRate::ToCAN<CANRollYawRateVehicle>(in);
	 msg.Cmd = static_cast<uint8_t>(CMD1);
         return msg;
      }

   };
   ASSERT_EIGHT(CANRollYawRateVehicle);

   struct CANRollYawRateImplement: public CANRollYawRate
   {
      enum { CMD1 = 0x7D };
      static inline CANRollYawRateImplement ToCAN(const RollYawRate& in)
      {
         CANRollYawRateImplement msg = CANRollYawRate::ToCAN<CANRollYawRateImplement>(in);
	 msg.Cmd = static_cast<uint8_t>(CMD1);
         return msg;
      }

   };
   ASSERT_EIGHT(CANRollYawRateImplement);

   ///////////////////////////////////////////
   struct PitchAltitude
   {
      deg32_t Pitch;
      m32_t Altitude;
   };

   struct CANPitchAltitude
   {
      enum { PGN = 0xFFFE };
      enum { CMD1 = 0x21 };
      enum { PRIORITY = 5 };

      uint8_t Cmd : 8;
      uint16_t Pitch : 16;
      uint16_t Altitude : 16;
      uint8_t Reserved1 : 8;
      uint8_t Reserved2 : 8;
      uint8_t Reserved3 : 8;

      static inline PitchAltitude FromCAN(const sc::CANMsg& msg)
      {
	 const auto& in = sc::ToMsg<CANPitchAltitude>(msg);

         PitchAltitude out;
         out.Pitch = cc::ScaledNoNaN(in.Pitch, 1.0f / 128.0f, 200.0f);
         out.Altitude = cc::ScaledNoNaN(in.Altitude, 0.125f, 2500.0f);
         return out;
      }

      static inline CANPitchAltitude ToCAN(const PitchAltitude& in)
      {
         CANPitchAltitude msg;
	 msg.Cmd = static_cast<uint8_t>(CMD1);
         msg.Pitch = static_cast<uint8_t>(CANPitchAltitude::CMD1);
         msg.Altitude = cc::Unscaled<uint16_t>(in.Pitch, 1.0f / 128.0f, 200.0f);
         msg.Reserved1 = 0xFF;
         msg.Reserved2 = 0xFF;
         msg.Reserved3 = 0xFF;
         return msg;
      }

   }__attribute__((packed));
   ASSERT_EIGHT(CANPitchAltitude);

   ///////////////////////////////////////////
   struct DirectionSpeed
   {
      deg32_t Bearing;
      kph32_t Speed;
      deg32_t Pitch;
      m32_t Altitude;
   };

   struct CANDirectionSpeed
   {
      enum { PGN = 0xFEE8 };
      enum { PRIORITY = 6 };

      uint16_t Bearing : 16;
      uint16_t Speed : 16;
      uint16_t Pitch : 16;
      uint16_t Altitude : 16;

      static inline DirectionSpeed FromCAN(const sc::CANMsg& msg)
      {
         const auto& in = sc::ToMsg<CANDirectionSpeed>(msg);
         DirectionSpeed out;
         out.Bearing = cc::ScaledNoNaN(in.Bearing, 1.0f / 128.0f);
         out.Speed = cc::ScaledNoNaN(in.Speed, 1.0f / 256.0f);
         out.Pitch = cc::ScaledNoNaN(in.Pitch, 1.0f / 128.0f, 200.0f);
         out.Altitude = cc::ScaledNoNaN(in.Altitude, 0.125f, 2500.0f);
         return out;
      }

      static inline CANDirectionSpeed ToCAN(const DirectionSpeed& in)
      {
         CANDirectionSpeed msg;
         msg.Bearing = cc::Unscaled<uint16_t>(in.Bearing, 1.0f / 128.0f);
         msg.Speed = cc::Unscaled<uint16_t>(in.Speed, 1.0f / 256.0f);
         msg.Pitch = cc::Unscaled<uint16_t>(in.Pitch, 1.0f / 128.0f, 200.0f);
         msg.Altitude = cc::Unscaled<uint16_t>(in.Altitude, 0.125f, 2500.0f);
         return msg;
      }

   };
   ASSERT_EIGHT(CANDirectionSpeed);

   ///////////////////////////////////////////
   struct BearingSpeed
   {
      deg32_t Bearing;
      kph32_t Speed;
   };

   struct CANBearingSpeed
   {
      enum { PGN = 0xFFFE };
      enum { CMD1 = 0x20 };
      enum { PRIORITY = 6 };
      uint8_t Cmd : 8;
      uint16_t Bearing : 16;
      uint16_t Speed : 16;
      uint8_t Reserved1 : 8;
      uint8_t Reserved2 : 8;
      uint8_t Reserved3 : 8;

      static inline BearingSpeed FromCAN(const sc::CANMsg& msg)
      {
         const auto& in = sc::ToMsg<CANBearingSpeed>(msg);
         BearingSpeed out;
         out.Bearing = cc::ScaledNoNaN(in.Bearing, 1.0f / 128.0f);
         out.Speed = cc::ScaledNoNaN(in.Speed, 1.0f / 256.0f);
         return out;
      }

      static inline CANBearingSpeed ToCAN(const BearingSpeed& in)
      {
         CANBearingSpeed msg;
         msg.Cmd = static_cast<uint8_t>(CANBearingSpeed::CMD1);
         msg.Bearing = cc::Unscaled<uint16_t>(in.Bearing, 1.0f / 128.0f);
         msg.Speed = cc::Unscaled<uint16_t>(in.Speed, 1.0f / 256.0f);
         msg.Reserved1 = 0xFF;
         msg.Reserved2 = 0xFF;
         msg.Reserved3 = 0xFF;
         return msg;
      }
   }__attribute__((packed));
   ASSERT_EIGHT(CANBearingSpeed);

   ///////////////////////////////////////////
   struct TimeDate
   {
      float Second;
      int Minute;
      int Hour;
      int Month;
      float Day;
      int Year;
      int MinuteOffset;
      int HourOffset;
   };

   struct CANTimeDate
   {
      enum { PGN = 0xFEE6 };
      enum { PRIORITY = 6 };

      uint8_t Second : 8;
      uint8_t Minute : 8;
      uint8_t Hour : 8;
      uint8_t Month : 8;
      uint8_t Day : 8;
      uint8_t Year : 8;
      uint8_t MinuteOffset : 8;
      uint8_t HourOffset : 8;

      static inline TimeDate FromCAN(const sc::CANMsg& msg)
      {
	 const auto& in = sc::ToMsg<CANTimeDate>(msg);
         TimeDate out;
         out.Second = cc::ScaledNoNaN(in.Second, 0.25f);
         out.Minute = static_cast<int>(in.Minute);
         out.Hour = static_cast<int>(in.Hour);
         out.Month = static_cast<int>(in.Month);
         out.Day = cc::ScaledNoNaN(in.Day, 0.25f);
         out.Year = static_cast<int>(in.Year) + 1985;
         out.MinuteOffset = static_cast<int>(in.MinuteOffset) - 125;
         out.HourOffset = static_cast<int>(in.HourOffset) - 125;
         return out;
      }

      static inline CANTimeDate ToCAN(const TimeDate& in)
      {
         CANTimeDate msg;
         msg.Second = cc::Unscaled<uint8_t>(in.Second, 0.25f);
         msg.Minute = static_cast<uint8_t>(in.Minute);
         msg.Hour = static_cast<uint8_t>(in.Hour);
         msg.Month = static_cast<uint8_t>(in.Month);
         msg.Day = cc::Unscaled<uint8_t>(in.Day, 0.25f);
         msg.Year = static_cast<uint8_t>(in.Year - 1985);
         msg.MinuteOffset = static_cast<uint8_t>(in.MinuteOffset + 125);
         msg.HourOffset = static_cast<uint8_t>(in.HourOffset + 125);
         return msg;
      }

   };
   ASSERT_EIGHT(CANTimeDate);

   ///////////////////////////////////////////
   enum ETerrainCompensationAlgorithmStatus
   {
	TCAS_SOLUTION_NA = 0,
	TCAS_LEVELING_IN_PROGRESS = 1,
	TCAS_YAW_ALIGNMENT_IN_PROGRESS = 2,
	TCAS_FINE_ALIGNMENT_IN_PROCESS = 3,
	TCAS_FINE_ALIGNMENT_COMPLETE = 4,
	TCAS_RESERVED1 = 5,
	TCAS_RESERVED2 = 6,
	TCAS_RESERVED3 = 7,
	TCAS_RESERVED4 = 8,
	TCAS_RESERVED5 = 9,
	TCAS_RESERVED6 = 10,
	TCAS_RESERVED7 = 11,
	TCAS_RESERVED8 = 12,
	TCAS_CALIBRATING = 13,
	TCAS_ERROR = 14,
	TCAS_NOT_AVAILABLE = 15
   };

   struct TerrainCompensation
   {
      EActive TCMode { AC_NotAvailable };
      uint8_t TCAccuracy { 0 };
      EDirection Direction { D_NotAvailable };
      EDirection CourseDirection { D_NotAvailable };
      EActive LowSpeedCapable { AC_NotAvailable };
      ms32_t SolutionProjectionTime { 0 };
      bool IsValid { false };
      ETerrainCompensationAlgorithmStatus AlgorithmStatus {TCAS_NOT_AVAILABLE};
   };

   struct CANTerrainCompensation
   {
      enum { PGN = 0xFFFF };
      enum { PRIORITY = 5 };
      
      uint8_t Cmd : 8;

      uint8_t TCMode : 2;
      uint8_t Reserved1 : 6;

      uint8_t TCAccuracy : 8;

      uint8_t LowSpeedCapable : 2;
      uint8_t Direction : 2;
      uint8_t CourseDirection : 2;
      uint8_t Reserved2 : 2;

      uint8_t SolutionProjectionTime : 8;

      uint8_t Reserved3 : 8;
      uint8_t Reserved4 : 8;

      uint8_t AlgorithmStatus : 4;
      uint8_t Reserved5 : 4;


      static inline TerrainCompensation FromCAN(const sc::CANMsg& msg)
      {
	 TerrainCompensation out;
         if(msg.Length >= 5) // workaround for bug in GPS simulator, which only sends 3 bytes
         {
	    const auto& in = sc::ToMsg<CANTerrainCompensation>(msg);
	    out.TCMode = static_cast<EActive>(in.TCMode);
	    out.TCAccuracy = in.TCAccuracy,
	    out.Direction = static_cast<EDirection>(in.Direction);
	    out.LowSpeedCapable = static_cast<EActive>(in.LowSpeedCapable);
	    out.SolutionProjectionTime = static_cast<ms32_t>(in.SolutionProjectionTime);
	    out.AlgorithmStatus = static_cast<ETerrainCompensationAlgorithmStatus>(in.AlgorithmStatus);
	    out.IsValid = true;
         }
         return out;
      }

   protected:
      template <typename T> static T ToCAN(const TerrainCompensation& in)
      {
         T out;
	 out.Cmd = 0xFF;
	 out.TCMode = static_cast<uint8_t>(in.TCMode);
	 out.Reserved1 = 0b111111;
	 out.TCAccuracy = in.TCAccuracy,
	 out.Direction = static_cast<uint8_t>(in.Direction);
	 out.Reserved2 = 0b11;
	 out.LowSpeedCapable = static_cast<uint8_t>(in.LowSpeedCapable);
	 out.SolutionProjectionTime = static_cast<uint8_t>(in.SolutionProjectionTime);
	 out.Reserved3 = 0xFF;
	 out.Reserved4 = 0xFF;
	 out.AlgorithmStatus = static_cast<uint8_t>(in.AlgorithmStatus);
	 out.Reserved5 = 0b1111;
         return out;
      }

   }__attribute__((packed));
   ASSERT_EIGHT(CANTerrainCompensation);

   struct CANTerrainCompensationVehicle: public CANTerrainCompensation
   {
      enum { CMD1 = 0xE0 };
      static inline CANTerrainCompensationVehicle ToCAN(const TerrainCompensation& in)
      {
         auto msg = CANTerrainCompensation::ToCAN<CANTerrainCompensationVehicle>(in);
	 msg.Cmd = static_cast<uint8_t>(CMD1);
         return msg;
      }
   }__attribute__((packed));
   ASSERT_EIGHT(CANTerrainCompensationVehicle);

   struct CANTerrainCompensationImplement: public CANTerrainCompensation
   {
      enum { CMD1 = 0x7C };
      static inline CANTerrainCompensationImplement ToCAN(const TerrainCompensation& in)
      {
         auto msg = CANTerrainCompensation::ToCAN<CANTerrainCompensationImplement>(in);
	 msg.Cmd = static_cast<uint8_t>(CMD1);
         return msg;
      }
   }__attribute__((packed));
   ASSERT_EIGHT(CANTerrainCompensationImplement);

   ///////////////////////////////////////////
   struct GpsStatus
   {
      EPositionMode Mode { PM_Undefined };
      ECorrectionState State { CS_NotAvailable };
      uint8_t VelocitySolutionSatellites { 0 };
      uint8_t PDOP { 0 };
      uint8_t HDOP { 0 };
      uint8_t VDOP { 0 };
      sec32_t CorrectionAge { 0 };
   };

   struct CANGpsStatus
   {
      enum { PGN = 0xFFFF };

      uint8_t Cmd : 8;
      uint8_t Mode : 4;
      uint8_t Reserved1 : 4;
      uint8_t State : 4;
      uint8_t Reserved2 : 4;
      uint8_t VelocitySolutionSatellites : 8;
      uint8_t PDOP : 8;
      uint8_t HDOP : 8;
      uint8_t VDOP : 8;
      uint8_t CorrectionAge : 8;

      static inline GpsStatus FromCAN(const sc::CANMsg& msg)
      {
	 const auto& in = sc::ToMsg<CANGpsStatus>(msg);
         GpsStatus out;
         out.Mode = static_cast<EPositionMode>(in.Mode);
         out.State = static_cast<ECorrectionState>(in.State);
         out.VelocitySolutionSatellites = in.VelocitySolutionSatellites;
         out.PDOP = static_cast<uint8_t>(cc::ScaledNoNaN(in.PDOP, 0.1));
         out.HDOP = static_cast<uint8_t>(cc::ScaledNoNaN(in.HDOP, 0.1));
         out.VDOP = static_cast<uint8_t>(cc::ScaledNoNaN(in.VDOP, 0.1));
         out.CorrectionAge = static_cast<sec32_t>(in.CorrectionAge);
         return out;
      }

   protected:
      template<typename T> static T ToCAN(const GpsStatus& in)
      {
         T out;
	 out.Cmd = 0xFF;
         out.Mode = static_cast<uint8_t>(in.Mode);
	 out.Reserved1 = 0b1111;
         out.State = static_cast<uint8_t>(in.State);
	 out.Reserved2 = 0b1111;
         out.VelocitySolutionSatellites = in.VelocitySolutionSatellites;
         out.PDOP = cc::UnscaledNoNaN<uint8_t>(in.PDOP, 0.1);
         out.HDOP = cc::UnscaledNoNaN<uint8_t>(in.HDOP, 0.1);
         out.VDOP = cc::UnscaledNoNaN<uint8_t>(in.VDOP, 0.1);
         out.CorrectionAge = static_cast<uint8_t>(in.CorrectionAge);
         return out;
      }

   };
   ASSERT_EIGHT(CANGpsStatus);

   struct CANGpsStatusVehicle: public CANGpsStatus
   {
      enum { CMD1 = 0x51 };
      enum { PRIORITY = 3 };

      static inline CANGpsStatusVehicle ToCAN(const GpsStatus& in)
      {
         auto msg = CANGpsStatus::ToCAN<CANGpsStatusVehicle>(in);
         msg.Cmd = static_cast<uint8_t>(CMD1);
         return msg;
      }

   };
   ASSERT_EIGHT(CANGpsStatusVehicle);

   struct CANGpsStatusImplement: public CANGpsStatus
   {
      enum { CMD1 = 0x7B };
      enum { PRIORITY = 7 };

      static inline CANGpsStatusImplement ToCAN(const GpsStatus& in)
      {
         auto msg = CANGpsStatus::ToCAN<CANGpsStatusImplement>(in);
         msg.Cmd = static_cast<uint8_t>(CMD1);
         return msg;
      }
   };
   ASSERT_EIGHT(CANGpsStatusImplement);

   ///////////////////////////////////////////
   struct SatellitesUsed
   {
      uint32_t SatellitesMask;
      lat64_t LatitudeEx;
      lon64_t LongitudeEx;
      mm32_t AltitudeEx;
   };

   struct CANSatellitesUsed
   {
      enum { PGN = 0xFFFF };
      enum { CMD1 = 0x52 };
      enum { PRIORITY = 3 };

      uint8_t Cmd : 8;
      uint32_t SatellitesMask : 32;
      uint8_t LatitudeEx : 8;
      uint8_t LongitudeEx : 8;
      uint8_t AltitudeEx : 8;

      static inline SatellitesUsed FromCAN(const sc::CANMsg& msg)
      {
	 const auto& in = sc::ToMsg<CANSatellitesUsed>(msg);
         SatellitesUsed out;
         out.SatellitesMask = in.SatellitesMask;
         out.LatitudeEx = cc::ScaledNoNaN(in.LatitudeEx, 0.0000001 / 256.0);
         out.LongitudeEx = cc::ScaledNoNaN(in.LongitudeEx, 0.0000001 / 256.0);
         out.AltitudeEx = cc::ScaledNoNaN(in.AltitudeEx, 0.4883f);
         return out;
      }

      static inline CANSatellitesUsed ToCAN(const SatellitesUsed& in)
      {
         CANSatellitesUsed msg;
         msg.Cmd = static_cast<uint8_t>(CANSatellitesUsed::CMD1);
         msg.SatellitesMask = in.SatellitesMask;
         msg.LatitudeEx = cc::UnscaledNoNaN<uint8_t>(in.LatitudeEx, 0.0000001 / 256.0);
         msg.LongitudeEx = cc::UnscaledNoNaN<uint8_t>(in.LongitudeEx, 0.0000001 / 256.0);
         msg.AltitudeEx = cc::UnscaledNoNaN<uint8_t>(in.AltitudeEx, 0.4883f);
         return msg;
      }

   }__attribute__((packed));
   ASSERT_EIGHT(CANSatellitesUsed);

   ///////////////////////////////////////////
   struct ReceiverInfo
   {
      uint16_t LicenseDays;
      uint8_t HardwareVersion; //scaled x10 from message definition to avoid having wrong decimal places from using float
      uint8_t SoftwareVersion;
      uint32_t SerialNumber;
      EReceiverVersion ReceiverVersion { RV_NotAvailable };
   };

   struct CANReceiverInfo
   {
      enum { PGN = 0xFFFF };
      enum { CMD1 = 0x54 };
      enum { PRIORITY = 6 };

      uint8_t Cmd : 8;
      uint16_t LicenseDays : 16;
      uint8_t HardwareVersion : 8; //scaled x10 from message definition to avoid having wrong decimal places from using float
      uint8_t SoftwareVersion : 8;
      uint16_t SerialNumberLSB : 16;
      uint8_t ReceiverVersion : 4;
      uint8_t SerialNumberMSB : 4;

      static inline ReceiverInfo FromCAN(const sc::CANMsg& msg)
      {
	 const auto& in = sc::ToMsg<CANReceiverInfo>(msg);
         ReceiverInfo out;
         out.LicenseDays = in.LicenseDays;
         out.HardwareVersion = in.HardwareVersion;
         out.SoftwareVersion = in.SoftwareVersion;
         out.SerialNumber = static_cast<uint32_t>((in.SerialNumberMSB << 16) | in.SerialNumberLSB);
         out.ReceiverVersion = static_cast<EReceiverVersion>(in.ReceiverVersion);
	 return out;
      }

      static inline CANReceiverInfo ToCAN(const ReceiverInfo& in)
      {
         CANReceiverInfo out;
         out.Cmd = static_cast<uint8_t>(CANReceiverInfo::CMD1);
         out.LicenseDays = cc::Unscaled<uint16_t>(in.LicenseDays, 1.0f);
         out.HardwareVersion = in.HardwareVersion;
         out.SoftwareVersion = in.SoftwareVersion;
         out.SerialNumberLSB = static_cast<uint16_t>(in.SerialNumber & 0xFFFF);
         out.ReceiverVersion = static_cast<uint8_t>(in.ReceiverVersion);
         out.SerialNumberMSB = static_cast<uint8_t>((in.SerialNumber & 0x0F0000) >> 16);
         return out;
      }

   }__attribute__((packed));
   ASSERT_EIGHT(CANReceiverInfo);

   ///////////////////////////////////////////
   struct HeadingMode
   {
      deg32_t Heading;
      EDifferentialSource NavigationMode { DS_NotAvailable };
      ELocked NavigationLocked { LO_NotAvaialble };
      cm32_t HorizontalErrorEstimate { 0.0 };
      sec32_t ActivationDelta { 0 };
      ECorrect Sequence { CO_NotAvaialble };
      ECorrect Decode { CO_NotAvaialble };
      mm32_t HorizontalErrorEstimateExtendedPrecision { 0.0 };
      ESolutionStatus SolutionStatus { SST_NotAvailable };
   };

   struct CANHeadingMode
   {
      enum { PGN = 0xFFFF };
      enum { CMD1 = 0xE3 };
      enum { PRIORITY = 7 };
      static constexpr mm32_t MAX_EXTENDED_PRECISION = 9.0;
      
      uint8_t Cmd : 8;

      uint16_t Heading :16;

      uint8_t NavigationMode : 8;

      uint8_t Reserved1 : 6;
      uint8_t NavigationLocked : 2;

      uint8_t HorizontalErrorEstimate : 8;

      uint8_t ActivationDelta : 4;
      uint8_t Sequence : 2;
      uint8_t Decode : 2;

      uint8_t HorizontalErrorEstimateExtendedPrecision : 4;
      uint8_t SolutionStatus : 4;

      static inline HeadingMode FromCAN(const sc::CANMsg& msg)
      {
	 const auto& in = sc::ToMsg<CANHeadingMode>(msg);
	 HeadingMode out;
         out.Heading = cc::ScaledNoNaN(in.Heading, 1.0f / 128.0f);
         out.NavigationMode = static_cast<EDifferentialSource>(in.NavigationMode);
         out.NavigationLocked = static_cast<ELocked>(in.NavigationLocked);
         out.HorizontalErrorEstimate = static_cast<cm32_t>(in.HorizontalErrorEstimate);
         out.ActivationDelta = static_cast<sec32_t>(in.ActivationDelta);
         out.Sequence = static_cast<ECorrect>(in.Sequence);
         out.Decode = static_cast<ECorrect>(in.Decode);
         out.HorizontalErrorEstimateExtendedPrecision = static_cast<mm32_t>(in.HorizontalErrorEstimateExtendedPrecision);
         out.SolutionStatus = static_cast<ESolutionStatus>(in.SolutionStatus);
	 return out;
      }

      static inline CANHeadingMode ToCAN(const HeadingMode& in)
      {
         CANHeadingMode out;
         out.Cmd = static_cast<uint8_t>(CANHeadingMode::CMD1);
         out.Heading = cc::Unscaled<uint16_t>(in.Heading, 1.0f / 128.0f);
         out.NavigationMode = static_cast<uint8_t>(in.NavigationMode);
         out.Reserved1 = 0b111111;
         out.NavigationLocked = static_cast<uint8_t>(in.NavigationLocked);
         out.HorizontalErrorEstimate = cc::Unscaled<uint8_t>(in.HorizontalErrorEstimate, 1.0);
         out.ActivationDelta = static_cast<uint8_t>(in.ActivationDelta);
         out.Sequence = static_cast<uint8_t>(in.Sequence);
         out.Decode = static_cast<uint8_t>(in.Decode);
         out.HorizontalErrorEstimateExtendedPrecision = cc::Unscaled<uint8_t>(in.HorizontalErrorEstimateExtendedPrecision, 1.0);
         out.SolutionStatus = static_cast<uint8_t>(in.SolutionStatus);
         return out;
      }

   }__attribute__((packed));
   ASSERT_EIGHT(CANHeadingMode);

   //////////////////////////////////////////////////////////////
   ///Class 3 Messages
   //////////////////////////////////////////////////////////////

   enum ESpeedSource //A.30.4
   {
      SPD_Wheel = 0,
      SPD_Ground = 1,
      SPD_Gps = 2,
      SPD_Blended = 3,
      SPD_Simulated = 4,
      SPD_Reserved1 = 5,
      SPD_Reserved2 = 6,
      SPD_NotAvailable = 7
   };


   enum ELimitStatus //A.19.11
   {
      LS_NotLimited = 0x00,
      LS_OperatorLimited = 0x01,
      LS_LimitedHigh = 0x02,
      LS_LimitedLow = 0x03,
      LS_Error = 0x06,
      LS_NotAvailable = 0x07
   };

      //B.28.1
   ///@todo: verify if the initialzation values are reasonable
   struct MachineSelectedSpeed
   {
      mps_t Speed = 0.f;
      m_t Distance = 0.f;
      uint8_t ExitCode = 0;
      EDirection Direction = D_Forward;
      ESpeedSource SpeedSource = SPD_NotAvailable;
      ELimitStatus SpeedLimitStatus = LS_NotAvailable;
      sc::addr_t Source = sc::NULL_ADDRESS;
   };

   struct CANMachineSelectedSpeed
   {
      enum { PGN = 0xF022 };
      enum { PRIORITY = 3 };

      uint16_t Speed : 16;
      uint32_t Distance : 32;
      uint8_t ExitCode : 6;
      uint8_t Reserved1 : 2;
      uint8_t Direction : 2;
      uint8_t SpeedSource : 3;
      uint8_t SpeedLimitStatus : 3;

      static MachineSelectedSpeed FromCAN(const sc::CANMsg& msg)
      {
	 const auto& in = sc::ToMsg<CANMachineSelectedSpeed>(msg);

         MachineSelectedSpeed out;
         out.Speed = cc::Scaled(in.Speed, 0.001f);
         out.Distance = cc::Scaled(in.Distance, 0.001f);
         out.ExitCode = in.ExitCode;
         out.Direction = in.Direction == D_Error ? D_NotAvailable : static_cast<EDirection>(in.Direction);
         out.SpeedSource = static_cast<ESpeedSource>(in.SpeedSource);
         out.SpeedLimitStatus = static_cast<ELimitStatus>(in.SpeedLimitStatus);
         out.Source = msg.Source;
         return out;
      }

      static CANMachineSelectedSpeed ToCAN(const MachineSelectedSpeed& in)
      {
         CANMachineSelectedSpeed out;
         out.Speed = cc::Unscaled<uint16_t>(in.Speed, 0.001f);
         out.Distance = cc::Unscaled<uint32_t>(in.Distance, 0.001f);
         out.ExitCode = in.ExitCode;
         out.Reserved1 = 0b11;
	 out.Direction = static_cast<uint8_t>(in.Direction);
	 out.SpeedSource = static_cast<uint8_t>(in.SpeedSource);
	 out.SpeedLimitStatus = static_cast<uint8_t>(in.SpeedLimitStatus);
         return out;
      }

   }__attribute__((packed));
   ASSERT_EIGHT(CANMachineSelectedSpeed);

   //B.3
   struct WheelBasedSpeedAndDistance
   {
      mps_t Speed = 0.0f;//A.8
      m_t Distance = 0.0f;//A.9 0.001 m per bit
      min_t MaximumPowerTime = 0;//A.12
      EDirection MachineDirection = D_NotAvailable;//A.10
      EOnOff KeySwitch = OO_NotAvailable;
      EEnable StartStop = EN_NotAvailable;//A.25.2
      EDirection OperatorDirection = D_NotAvailable;//A.31
   };

   struct CANWheelBasedSpeedAndDistance
   {
      enum { PGN = 0xFE48 };
      enum { PRIORITY = 3 };

      uint16_t Speed : 16;
      uint32_t Distance : 32;
      uint8_t MaximumPowerTime : 8;
      uint8_t MachineDirection : 2;
      uint8_t KeySwitch : 2;
      uint8_t StartStop : 2;
      uint8_t OperatorDirection : 2;

      static WheelBasedSpeedAndDistance FromCAN(const sc::CANMsg& msg)
      {
	 const auto& in = sc::ToMsg<CANWheelBasedSpeedAndDistance>(msg);
         WheelBasedSpeedAndDistance out;

         out.Speed = cc::Scaled(in.Speed, 0.001f);
         out.Distance = cc::Scaled(in.Distance, 0.001f);
         out.MaximumPowerTime = static_cast<min_t>(in.MaximumPowerTime);
         out.MachineDirection = static_cast<EDirection>(in.MachineDirection);
         out.KeySwitch = static_cast<EOnOff>(in.KeySwitch);
         out.StartStop = static_cast<EEnable>(in.StartStop);
         out.OperatorDirection = static_cast<EDirection>(in.OperatorDirection);
         return out;
      }

      static CANWheelBasedSpeedAndDistance ToCAN(const WheelBasedSpeedAndDistance& in)
      {
         CANWheelBasedSpeedAndDistance out;
         out.Speed = cc::Unscaled<uint16_t>(in.Speed, 0.001f);
         out.Distance = cc::Unscaled<uint32_t>(in.Distance, 0.001f);
         out.MaximumPowerTime = static_cast<uint8_t>(in.MaximumPowerTime);
         out.MachineDirection = static_cast<uint8_t>(in.MachineDirection);
         out.KeySwitch = static_cast<uint8_t>(in.KeySwitch);
         out.StartStop = static_cast<uint8_t>(in.StartStop);
         out.OperatorDirection = static_cast<uint8_t>(in.OperatorDirection);
         return out;
      }

   }__attribute__((packed));
   ASSERT_EIGHT(CANWheelBasedSpeedAndDistance);

   //////////////////////////////////////////////////////////////
   ///Datum
   //////////////////////////////////////////////////////////////

   enum EDatumType
   {
      DT_Generic_Deere_RTK_Base = 0,
      DT_Generic_3rd_Party_RTK_Single_Base = 1,
      DT_Generic_3rd_Party_RTK_Network_Base = 2,
      DT_High_Accuracy_SF_Navigation_ITRF_2014 = 3,
      DT_Reserved4 = 4,
      DT_Reserved5 = 5,
      DT_Reserved6 = 6,
      DT_Reserved7 = 7,
      DT_Reserved8 = 8,
      DT_Reserved9 = 9,
      DT_Reserved10 = 10,
      DT_Reserved11 = 11,
      DT_Reserved12 = 12,
      DT_Reserved13 = 13,
      DT_Error = 14,
      DT_NotAvailable = 15
   };

   enum EDatumOffsetStatus
   {
      DOS_Pending = 0,
      DOS_Available = 1,
      DOS_Off = 2,
      DOS_NotSupported = 3,
      DOS_Reserved4 = 4,
      DOS_Reserved5 = 5,
      DOS_Reserved6 = 6,
      DOS_Reserved7 = 7,
      DOS_Reserved8 = 8,
      DOS_Reserved9 = 9,
      DOS_Reserved10 = 10,
      DOS_Reserved11 = 11,
      DOS_Reserved12 = 12,
      DOS_Reserved13 = 13,
      DOS_Error = 14,
      DOS_NotAvailable = 15
   };

   enum EDatumMatch
   {
      DM_Mismatch = 0,
      DM_Matches = 1,
      DM_Error = 2,
      DM_NotAvailable = 3
   };

   struct ReceiverDatum
   {
      EDatumType ActiveDatum = DT_NotAvailable;
      days_t BaseEpochTime = 0;//Since Jan 1,1970
      lat64_t BaseLat = 0.0;
      lon64_t BaseLon = 0.0;
      m32_t BaseHeight = 0.0f;
      EDatumType ReferenceDatum = DT_NotAvailable;
      EDatumOffsetStatus OffsetStatus = DOS_NotAvailable;
      days_t RefererenceEpochTime = 0;//Since Jan 1,1970
      lat64_t OffsetLat = 0.0;
      lon64_t OffsetLon = 0.0;
      m32_t OffsetHeight = 0.0f;
      m32_t OffsetHorzizontalUncertainty = 0.0f;
      m32_t OffsetVerticalUncertainty = 0.0f;
      days_t CreationDate = 0;//Since Jan 1,1970
      sc::addr_t SourceAddr = sc::NULL_ADDRESS;
      sc::node_t SourceDeviceID = sc::INVALID_NODE;
      EDatumMatch MatchesExpected = DM_NotAvailable;
   };

   struct CANReceiverDatum
   {
      enum { PGN = 0xEF00 };
      enum {PRIORITY = 6};
      enum { CMD1 = 0xF9 };
      enum { CMD2 = 0xA9 };

      uint16_t CmdBytes : 16; //Bytes 1-2

      uint8_t ActiveDatum : 4; //Byte 3
      uint8_t Reserved1 : 4;

      uint16_t BaseEpochTime : 16; //Bytes 4-5
      uint64_t BaseLat : 40; //Bytes 6-10
      uint64_t BaseLon : 40; //Bytes 11-15
      uint32_t BaseHeight : 24;//Bytes 16-18

      uint8_t ReferenceDatum : 4;//Byte 19
      uint8_t OffsetStatus : 4;

      uint16_t RefererenceEpochTime : 16;//Bytes 20-21
      uint32_t OffsetLat : 24; //Bytes 22-24
      uint32_t OffsetLon : 24; //Bytes 25-27
      uint32_t OffsetHeight : 24; //Bytes 28-30
      uint8_t OffsetHorzizontalUncertainty : 8;//Byte 31
      uint8_t OffsetVerticalUncertainty : 8; //Byte 32
      uint16_t CreationDate : 16; //Bytes 33-34
      uint64_t Reserved2 : 64; //Bytes 35-42

      static inline ReceiverDatum FromCAN(const sc::CANMsg& msg)
      {
         const CANReceiverDatum& in = sc::ToMsg<CANReceiverDatum>(msg);
         ReceiverDatum out;
         out.ActiveDatum = static_cast<EDatumType>(in.ActiveDatum);
         out.BaseEpochTime = static_cast<days_t>(in.BaseEpochTime);
         out.BaseLat = cc::ScaledNoNaN64(in.BaseLat, 0.000000001, 180.0);
         out.BaseLon = cc::ScaledNoNaN64(in.BaseLon, 0.000000001, 180.0);
         out.BaseHeight = cc::ScaledNoNaN(in.BaseHeight, 0.001f, 1000.0f);
         out.ReferenceDatum = static_cast<EDatumType>(in.ReferenceDatum);
         out.OffsetStatus = static_cast<EDatumOffsetStatus>(in.OffsetStatus);
         out.RefererenceEpochTime = static_cast<days_t>(in.RefererenceEpochTime);
         out.OffsetLat = cc::ScaledNoNaN64(in.OffsetLat, 0.000000001, 0.008);
         out.OffsetLon = cc::ScaledNoNaN64(in.OffsetLon, 0.000000001, 0.008);
         out.OffsetHeight = cc::ScaledNoNaN(in.OffsetHeight, 0.001f, 8000.0f);
         out.OffsetHorzizontalUncertainty = cc::ScaledNoNaN(in.OffsetHorzizontalUncertainty, 0.001f);
         out.OffsetVerticalUncertainty = cc::ScaledNoNaN(in.OffsetVerticalUncertainty, 0.001f);
         out.CreationDate = static_cast<days_t>(in.CreationDate);
         out.SourceAddr = msg.Source;
         out.SourceDeviceID = msg.SourceDeviceID;
         out.MatchesExpected = DM_NotAvailable;
         return out;
      }

      static inline CANReceiverDatum ToCAN(const ReceiverDatum& in)
      {
         CANReceiverDatum out;
         out.CmdBytes = cc::ToCMD16<CANReceiverDatum>();
         out.ActiveDatum = static_cast<uint8_t>(in.ActiveDatum);
         out.Reserved1 = 0b1111;
         out.BaseEpochTime = static_cast<uint16_t>(in.BaseEpochTime);
         out.BaseLat = cc::UnscaledNoNaN64<uint64_t>(in.BaseLat, 0.000000001, 180.0);
         out.BaseLon = cc::UnscaledNoNaN64<uint64_t>(in.BaseLon, 0.000000001, 180.0);
         out.BaseHeight = cc::UnscaledNoNaN<uint32_t>(in.BaseHeight, 0.001f, 1000.0f);
         out.ReferenceDatum = static_cast<uint8_t>(in.ReferenceDatum);
         out.OffsetStatus = static_cast<uint8_t>(in.OffsetStatus);
         out.RefererenceEpochTime = static_cast<uint16_t>(in.RefererenceEpochTime);
         out.OffsetLat = cc::UnscaledNoNaN64<uint32_t>(in.OffsetLat, 0.000000001, 0.008);
         out.OffsetLon = cc::UnscaledNoNaN64<uint32_t>(in.OffsetLon, 0.000000001, 0.008);
         out.OffsetHeight = cc::UnscaledNoNaN<uint32_t>(in.OffsetHeight, 0.001f, 8000.0f);
         out.OffsetHorzizontalUncertainty = cc::UnscaledNoNaN<uint8_t>(in.OffsetHorzizontalUncertainty, 0.001f);
         out.OffsetVerticalUncertainty = cc::UnscaledNoNaN<uint8_t>(in.OffsetVerticalUncertainty, 0.001f);
         out.CreationDate = static_cast<uint16_t>(in.CreationDate);
         out.Reserved2 = 0xFFFFFFFFFFFFFFFF;
         return out;
      }

   }
   __attribute__((packed));
   ASSERT_SIZE(CANReceiverDatum, 42);
}
