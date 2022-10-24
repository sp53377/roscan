///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <GpsService/GpsMessages.h>
#include <GpsService/GpsTypes.h>
#include <can_interfaces/msg/bearing_speed.hpp>
#include <can_interfaces/msg/differential_status.hpp>
#include <can_interfaces/msg/direction_speed.hpp>
#include <can_interfaces/msg/gps_device.hpp>
#include <can_interfaces/msg/gps_epoch.hpp>
#include <can_interfaces/msg/gps_ex.hpp>
#include <can_interfaces/msg/gps_status.hpp>
#include <can_interfaces/msg/heading_mode.hpp>
#include <can_interfaces/msg/machine_selected_speed.hpp>
#include <can_interfaces/msg/pitch_altitude.hpp>
#include <can_interfaces/msg/receiver_info.hpp>
#include <can_interfaces/msg/roll_yaw_rate.hpp>
#include <can_interfaces/msg/satellites_used.hpp>
#include <can_interfaces/msg/terrain_compensation.hpp>
#include <can_interfaces/msg/time_date.hpp>
#include <can_interfaces/msg/vehicle_position.hpp>
#include <can_interfaces/msg/wheel_based_speed_and_distance.hpp>

namespace bridge
{

template <typename rostype> void CanDataFromROS(const rostype & in, uint8_t* pSource, int64_t* pTimestamp)
{
	if(pSource){ *pSource = in.can_data.source;}
	if(pTimestamp){*pTimestamp = in.can_data.timestamp;}
}

struct VehiclePosition
{
  using cantype = gps::CANVehiclePosition;
  using cpptype = gps::VehiclePosition;
  using rostype = can_interfaces::msg::VehiclePosition;
  static constexpr const char * topic = "/gps/VehiclePosition";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.latitude = in.Latitude;
    out.longitude = in.Longitude;
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out.Latitude = in.latitude;
    out.Longitude = in.longitude;
    return out;
  }
};

struct DifferentialStatus
{
  using cantype = gps::CANDifferentialStatus;
  using cpptype = gps::DifferentialStatus;
  using rostype = can_interfaces::msg::DifferentialStatus;
  static constexpr const char * topic = "/gps/DifferentialStatus";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.diff_locked = in.DiffLocked;
    out.augmentation_type = in.AugmentationType;
    out.signal_to_noise_ratio = in.SignalToNoiseRatio;
    out.differential_source = in.DiffSource;
    out.accuracy = in.Accuracy;
    out.sf_active = in.SFActive;
    out.sf2_active = in.SF2Active;
    out.rtk_active = in.RTKActive;
    out.dual_frequency = in.DualFrequency;
    out.sf_activation = in.SFActivation;
    out.sf2_license = in.SF2License;
    out.rtk_activation = in.RTKActivation;
    out.at_capable = in.ATCapable;
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out.DiffLocked = static_cast<ELocked>(in.diff_locked);
    out.AugmentationType = static_cast<gps::EAugmentationType>(in.augmentation_type);
    out.SignalToNoiseRatio = static_cast<db32_t>(in.signal_to_noise_ratio);
    out.DiffSource = static_cast<gps::EDifferentialSource>(in.differential_source);
    out.Accuracy = static_cast<uint8_t>(in.accuracy);
    out.SFActive = static_cast<EActive>(in.sf_active);
    out.SF2Active = static_cast<EActive>(in.sf2_active);
    out.RTKActive = static_cast<EActive>(in.rtk_active);
    out.DualFrequency = static_cast<EActive>(in.dual_frequency);
    out.SFActivation = static_cast<EActive>(in.sf_activation);
    out.SF2License = static_cast<EActive>(in.sf2_license);
    out.RTKActivation = static_cast<EActive>(in.rtk_activation);
    out.ATCapable = static_cast<gps::EATCapable>(in.at_capable);
    return out;
  }
};

struct RollYawRate
{
  using cpptype = gps::RollYawRate;
  using rostype = can_interfaces::msg::RollYawRate;

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.cos_roll_angle_rad = in.CosRollAngle;
    out.roll_angle_deg = in.RollAngle;
    out.yaw_rate_dps = in.YawRate;
    out.rtk_distance_km = in.RTKDistance;
    out.altitude_error_mm = in.AltitudeError;
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out.CosRollAngle = in.cos_roll_angle_rad;
    out.RollAngle = in.roll_angle_deg;
    out.YawRate = in.yaw_rate_dps;
    out.RTKDistance = in.rtk_distance_km;
    out.AltitudeError = in.altitude_error_mm;
    return out;
  }
};

struct RollYawRateVehicle : public RollYawRate
{
  using cantype = gps::CANRollYawRateVehicle;
  static constexpr const char * topic = "/gps/RollYawRateVehicle";
};

struct RollYawRateImplement : public RollYawRate
{
  using cantype = gps::CANRollYawRateImplement;
  static constexpr const char * topic = "/gps/RollYawRateImplement";
};

struct PitchAltitude
{
  using cantype = gps::CANPitchAltitude;
  using cpptype = gps::PitchAltitude;
  using rostype = can_interfaces::msg::PitchAltitude;
  static constexpr const char * topic = "/gps/PitchAltitude";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.pitch_deg = in.Pitch;
    out.altitude_m = in.Pitch;
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out.Pitch = in.pitch_deg;
    out.Altitude = in.altitude_m;
    return out;
  }
};

struct DirectionSpeed
{
  using cantype = gps::CANDirectionSpeed;
  using cpptype = gps::DirectionSpeed;
  using rostype = can_interfaces::msg::DirectionSpeed;
  static constexpr const char * topic = "/gps/DirectionSpeed";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.bearing_deg = in.Bearing;
    out.speed_kph = in.Speed;
    out.pitch_deg = in.Pitch;
    out.altitude_m = in.Altitude;
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out.Bearing = in.bearing_deg;
    out.Speed = in.speed_kph;
    out.Pitch = in.pitch_deg;
    out.Altitude = in.altitude_m;
    return out;
  }
};

struct BearingSpeed
{
  using cantype = gps::CANBearingSpeed;
  using cpptype = gps::BearingSpeed;
  using rostype = can_interfaces::msg::BearingSpeed;
  static constexpr const char * topic = "/gps/BearingSpeed";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.bearing_deg = in.Bearing;
    out.speed_kph = in.Speed;
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out.Bearing = in.bearing_deg;
    out.Speed = in.speed_kph;
    return out;
  }
};

struct TimeDate
{
  using cantype = gps::CANTimeDate;
  using cpptype = gps::TimeDate;
  using rostype = can_interfaces::msg::TimeDate;
  static constexpr const char * topic = "/gps/TimeDate";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.second = in.Second;
    out.minute = in.Minute;
    out.hour = in.Hour;
    out.month = in.Month;
    out.day = in.Day;
    out.year = in.Year;
    out.minute_offset = in.MinuteOffset;
    out.hour_offset = in.HourOffset;
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out.Second = in.second;
    out.Minute = in.minute;
    out.Hour = in.hour;
    out.Month = in.month;
    out.Day = in.day;
    out.Year = in.year;
    out.MinuteOffset = in.minute_offset;
    out.HourOffset = in.hour_offset;
    return out;
  }
};

struct TerrainCompensation
{
  using cpptype = gps::TerrainCompensation;
  using rostype = can_interfaces::msg::TerrainCompensation;
  static constexpr const char * topic = "/gps/TerrainCompensation";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.tc_mode = in.TCMode;
    out.tc_accuracy = in.TCAccuracy;
    out.direction = in.Direction;
    out.course_direction = in.CourseDirection;
    out.low_speed_capable = in.LowSpeedCapable;
    out.solution_projection_time_ms = in.SolutionProjectionTime;
    out.is_valid = in.IsValid;
    out.algorithm_status = in.AlgorithmStatus;
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out.TCMode = static_cast<EActive>(in.tc_mode);
    out.TCAccuracy = in.tc_accuracy;
    out.Direction = static_cast<EDirection>(in.direction);
    out.CourseDirection = static_cast<EDirection>(in.course_direction);
    out.LowSpeedCapable = static_cast<EActive>(in.low_speed_capable);
    out.SolutionProjectionTime = in.solution_projection_time_ms;
    out.IsValid = in.is_valid;
    out.AlgorithmStatus = static_cast<gps::ETerrainCompensationAlgorithmStatus>(in.algorithm_status);
    return out;
  }
};

struct TerrainCompensationVehicle : public TerrainCompensation
{
  using cantype = gps::CANTerrainCompensationVehicle;
  static constexpr const char * topic = "/gps/TerrainCompensationVehicle";
};

struct TerrainCompensationImplement : public TerrainCompensation
{
  using cantype = gps::CANTerrainCompensationImplement;
  static constexpr const char * topic = "/gps/TerrainCompensationImplement";
};

struct GpsStatus
{
  using cantype = gps::CANGpsStatus;
  using cpptype = gps::GpsStatus;
  using rostype = can_interfaces::msg::GpsStatus;
  static constexpr const char * topic = "/gps/GpsStatus";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.mode = in.Mode;
    out.state = in.State;
    out.velocity_solution_satellites = in.VelocitySolutionSatellites;
    out.pdop = in.PDOP;
    out.hdop = in.HDOP;
    out.vdop = in.VDOP;
    out.correction_age_sec = in.CorrectionAge;
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out.Mode = static_cast<gps::EPositionMode>(in.mode);
    out.State = static_cast<gps::ECorrectionState>(in.state);
    out.VelocitySolutionSatellites = in.velocity_solution_satellites;
    out.PDOP = in.pdop;
    out.HDOP = in.hdop;
    out.VDOP = in.vdop;
    out.CorrectionAge = in.correction_age_sec;
    return out;
  }
};

struct GpsStatusVehicle : public GpsStatus
{
  using cantype = gps::CANGpsStatusVehicle;
  static constexpr const char * topic = "/gps/GpsStatusVehicle";
};

struct GpsStatusImplement : public GpsStatus
{
  using cantype = gps::CANGpsStatusImplement;
  static constexpr const char * topic = "/gps/GpsStatusImplement";
};

struct SatellitesUsed
{
  using cantype = gps::CANSatellitesUsed;
  using cpptype = gps::SatellitesUsed;
  using rostype = can_interfaces::msg::SatellitesUsed;
  static constexpr const char * topic = "/gps/SatellitesUsed";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.satellites_mask = in.SatellitesMask;
    out.latitude_ex_deg = in.LatitudeEx;
    out.longitude_ex_deg = in.LongitudeEx;
    out.altitude_ex_mm = in.AltitudeEx;
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out.SatellitesMask = in.satellites_mask;
    out.LatitudeEx = in.latitude_ex_deg;
    out.LongitudeEx = in.longitude_ex_deg;
    out.AltitudeEx = in.altitude_ex_mm;
    return out;
  }
};

struct ReceiverInfo
{
  using cantype = gps::CANReceiverInfo;
  using cpptype = gps::ReceiverInfo;
  using rostype = can_interfaces::msg::ReceiverInfo;
  static constexpr const char * topic = "/gps/ReceiverInfo";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.license_days = in.LicenseDays;
    out.hardware_version = in.HardwareVersion;
    out.software_version = in.SoftwareVersion;
    out.serial_number = in.SerialNumber;
    out.receiver_version = in.ReceiverVersion;
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out.LicenseDays = in.license_days;
    out.HardwareVersion = in.hardware_version;
    out.SoftwareVersion = in.software_version;
    out.SerialNumber = in.serial_number;
    out.ReceiverVersion = static_cast<gps::EReceiverVersion>(in.receiver_version);
    return out;
  }
};

struct HeadingMode
{
  using cantype = gps::CANHeadingMode;
  using cpptype = gps::HeadingMode;
  using rostype = can_interfaces::msg::HeadingMode;
  static constexpr const char * topic = "/gps/HeadingMode";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.heading_deg = in.Heading;
    out.navigation_mode = in.NavigationMode;
    out.navigation_locked = in.NavigationLocked;
    out.horizontal_error_estimate_cm = in.HorizontalErrorEstimate;
    out.activation_delta_sec = in.ActivationDelta;
    out.sequence = in.Sequence;
    out.decode = in.Decode;
    out.horizontal_error_estimate_extended_precision_mm = in.HorizontalErrorEstimateExtendedPrecision;
    out.solution_status = in.SolutionStatus;
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out.Heading = in.heading_deg;
    out.NavigationMode = static_cast<gps::EDifferentialSource>(in.navigation_mode);
    out.NavigationLocked = static_cast<ELocked>(in.navigation_locked);
    out.HorizontalErrorEstimate = in.horizontal_error_estimate_cm;
    out.ActivationDelta = in.activation_delta_sec;
    out.Sequence = static_cast<ECorrect>(in.sequence);
    out.Decode = static_cast<ECorrect>(in.decode);
    out.HorizontalErrorEstimateExtendedPrecision = in.horizontal_error_estimate_extended_precision_mm;
    out.SolutionStatus = static_cast<gps::ESolutionStatus>(in.solution_status);
    return out;
  }
};

struct MachineSelectedSpeed
{
  using cantype = gps::CANMachineSelectedSpeed;
  using cpptype = gps::MachineSelectedSpeed;
  using rostype = can_interfaces::msg::MachineSelectedSpeed;
  static constexpr const char * topic = "/gps/MachineSelectedSpeed";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.direction = static_cast<uint8_t>(in.Direction);
    out.distance_m = in.Distance;
    out.exit_code = in.ExitCode;
    out.speed_mps = in.Speed;
    out.speed_limit_status = static_cast<uint8_t>(in.SpeedLimitStatus);
    out.speed_source = static_cast<uint8_t>(in.SpeedSource);
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out.Direction = static_cast<EDirection>(in.direction);
    out.Distance = in.distance_m;
    out.ExitCode = in.exit_code;
    out.Speed = in.speed_mps;
    out.SpeedLimitStatus = static_cast<gps::ELimitStatus>(in.speed_limit_status);
    out.SpeedSource = static_cast<gps::ESpeedSource>(in.speed_source);
    return out;
  }
};

struct WheelBasedSpeedAndDistance
{
  using cantype = gps::CANWheelBasedSpeedAndDistance;
  using cpptype = gps::WheelBasedSpeedAndDistance;
  using rostype = can_interfaces::msg::WheelBasedSpeedAndDistance;
  static constexpr const char * topic = "/gps/WheelBasedSpeedAndDistance";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.distance_m = in.Distance;
    out.key_switch = static_cast<uint8_t>(in.KeySwitch);
    out.machine_direction = static_cast<uint8_t>(in.MachineDirection);
    out.maximum_power_time_min = in.MaximumPowerTime;
    out.operator_direction = static_cast<uint8_t>(in.OperatorDirection);
    out.speed_mps = in.Speed;
    out.start_stop = static_cast<uint8_t>(in.StartStop);
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out.Distance = in.distance_m;
    out.KeySwitch = static_cast<EOnOff>(in.key_switch);
    out.MachineDirection = static_cast<EDirection>(in.machine_direction);
    out.MaximumPowerTime = in.maximum_power_time_min;
    out.OperatorDirection = static_cast<EDirection>(in.operator_direction);
    out.Speed = in.speed_mps;
    out.StartStop = static_cast<EEnable>(in.start_stop);
    return out;
  }
};

//////////////////////////////////
////GPS Types
//////////////////////////////////
struct GpsEpoch
{
  using cpptype = gps::GpsEpoch;
  using rostype = can_interfaces::msg::GpsEpoch;
  static constexpr const char * topic = "/gps/GpsEpoch";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.can_data.source = in.Source;
    out.can_data.timestamp = in.Timestamp;
    out.latitude = in.Position.Lat;
    out.longitude = in.Position.Lon;
    out.elevation_m = in.Position.Elevation;
    out.accuracy = in.Accuracy;
    out.cos_roll_angle_rad = in.CosRollAngle;
    out.roll_angle_deg = in.RollAngle;
    out.yaw_rate_dps = in.YawRate;
    out.pitch_deg = in.Pitch;
    out.altitude_m = in.Altitude;
    out.speed_kph = in.Speed;
    out.bearing_deg = in.Bearing;
    out.direction = static_cast<uint8_t>(in.Direction);
    out.diff_locked = static_cast<uint8_t>(in.DiffLocked);
    out.pdop = in.PDOP;
    out.hdop = in.HDOP;
    out.vdop = in.VDOP;
    out.serial_number = in.SerialNumber;
    return out;
  }

  static inline cpptype FromROS(const rostype & in)
  {
    cpptype out;
    out.Timestamp = in.can_data.timestamp;
    out.Source = in.can_data.source;
    out.Position.Lat = in.latitude;
    out.Position.Lon = in.longitude;
    out.Position.Elevation = in.elevation_m;
    out.Accuracy = in.accuracy;
    out.CosRollAngle = in.cos_roll_angle_rad;
    out.RollAngle = in.roll_angle_deg;
    out.YawRate = in.yaw_rate_dps;
    out.Pitch = in.pitch_deg;
    out.Altitude = in.altitude_m;
    out.Speed = in.speed_kph;
    out.Bearing = in.bearing_deg;
    out.Direction = static_cast<EDirection>(in.diff_locked);
    out.DiffLocked = static_cast<ELocked>(in.diff_locked);
    out.PDOP = in.pdop;
    out.HDOP = in.hdop;
    out.VDOP = in.vdop;
    out.SerialNumber = in.serial_number;

    return out;
  }
};

struct GpsDevice
{
  using cpptype = gps::GpsDevice;
  using rostype = can_interfaces::msg::GpsDevice;
  static constexpr const char * topic = "/gps/GpsDevice";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.can_data.source = in.Source;
    out.can_data.timestamp = 0;
    out.diff_source = static_cast<uint8_t>(in.DiffSource);
    out.sf_active = static_cast<uint8_t>(in.SFActive);
    out.sf2_active = static_cast<uint8_t>(in.SF2Active);
    out.rtk_active = static_cast<uint8_t>(in.RTKActive);
    out.dual_frequency = static_cast<uint8_t>(in.DualFrequency);
    out.sf_activation = static_cast<uint8_t>(in.SFActivation);
    out.sf2_license = static_cast<uint8_t>(in.SF2License);
    out.rtk_activation = static_cast<uint8_t>(in.RTKActivation);
    out.at_capable = static_cast<uint16_t>(in.ATCapable);
    out.tc_mode = static_cast<uint8_t>(in.TCMode);
    out.mode = static_cast<uint8_t>(in.Mode);
    out.license_days = in.LicenseDays;
    out.hardware_version = in.HardwareVersion;
    out.software_version = in.SoftwareVersion;
    out.serial_number = in.SerialNumber;
    out.receiver_version = static_cast<uint8_t>(in.ReceiverVersion);
    out.type = static_cast<uint8_t>(in.Type);
    return out;
  }

  static inline cpptype FromROS(const rostype & in)
  {
    cpptype out;

    out.Source = in.can_data.source;
    //out.can_data.timestamp = 0;
    out.DiffSource = static_cast<gps::EDifferentialSource>(in.diff_source);
    out.SFActive = static_cast<EActive>(in.sf_active);
    out.SF2Active = static_cast<EActive>(in.sf2_active);
    out.RTKActive = static_cast<EActive>(in.rtk_active);
    out.DualFrequency = static_cast<EActive>(in.dual_frequency);
    out.SFActivation = static_cast<EActive>(in.sf_activation);
    out.SF2License = static_cast<EActive>(in.sf2_license);
    out.RTKActivation = static_cast<EActive>(in.rtk_activation);
    out.ATCapable = static_cast<gps::EATCapable>(in.at_capable);
    out.TCMode = static_cast<EActive>(in.tc_mode);
    out.Mode = static_cast<gps::EPositionMode>(in.mode);
    out.LicenseDays = in.license_days;
    out.HardwareVersion = in.hardware_version;
    out.SoftwareVersion = in.software_version;
    out.SerialNumber = in.serial_number;
    out.ReceiverVersion = static_cast<gps::EReceiverVersion>(in.receiver_version);
    out.Type = static_cast<EReceiverType>(in.type);

    return out;
  }
};

struct GpsEx
{
  using cpptype = gps::GpsEx;
  using rostype = can_interfaces::msg::GpsEx;
  static constexpr const char * topic = "/gps/GpsEx";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.can_data.source = in.Source;
    out.can_data.timestamp = in.Timestamp;

    out.navigation_locked = static_cast<uint8_t>(in.NavigationLocked);
    out.activation_delta_sec = in.ActivationDelta;
    out.navigation_mode = static_cast<uint8_t>(in.NavigationMode);
    out.velocity_solution_satellites = in.VelocitySolutionSatellites;
    out.state = static_cast<uint8_t>(in.State);
    out.satellites_mask = in.SatellitesMask;
    out.tc_accuracy = in.TCAccuracy;
    out.signal_to_noise_ratio_db = in.SignalToNoiseRatio;
    out.rtk_distance_km = in.RTKDistance;
    out.altitude_error_mm = in.AltitudeError;
    out.pitch_deg = in.Pitch;
    out.altitude_m = in.Altitude;
    out.bearing_deg = in.Bearing;
    out.speed_kph = in.Speed;
    out.solution_projection_time_ms = in.SolutionProjectionTime;
    out.correction_age_sec = in.CorrectionAge;
    out.horizontal_error_estimate_cm = in.HorizontalErrorEstimate;
    out.sequence = static_cast<uint8_t>(in.Sequence);
    out.decode = static_cast<uint8_t>(in.Decode);
    out.solution_status = static_cast<uint8_t>(in.SolutionStatus);
    out.low_speed_capable = static_cast<uint8_t>(in.LowSpeedCapable);
    out.serial_number = in.SerialNumber;
    return out;
  }

  static inline cpptype FromROS(const rostype & in)
  {
    cpptype out;
    out.NavigationLocked = static_cast<ELocked>(in.navigation_locked);
    out.ActivationDelta = in.activation_delta_sec;
    out.NavigationMode = static_cast<gps::EDifferentialSource>(in.navigation_mode);
    out.VelocitySolutionSatellites = in.velocity_solution_satellites;
    out.State = static_cast<gps::ECorrectionState>(in.state);
    out.SatellitesMask = in.satellites_mask;
    out.TCAccuracy = in.tc_accuracy;
    out.SignalToNoiseRatio = in.signal_to_noise_ratio_db;
    out.RTKDistance = in.rtk_distance_km;
    out.AltitudeError = in.altitude_error_mm;
    out.Pitch = in.pitch_deg;
    out.Altitude = in.altitude_m;
    out.Bearing = in.bearing_deg;
    out.Speed = in.speed_kph;
    out.SolutionProjectionTime = in.solution_projection_time_ms;
    out.CorrectionAge = in.correction_age_sec;
    out.HorizontalErrorEstimate = in.horizontal_error_estimate_cm;
    out.Sequence = static_cast<ECorrect>(in.sequence);
    out.Decode = static_cast<ECorrect>(in.decode);
    out.SolutionStatus = static_cast<gps::ESolutionStatus>(in.solution_status);
    out.LowSpeedCapable = static_cast<EActive>(in.low_speed_capable);
    out.SerialNumber = in.serial_number;
    out.Source = in.can_data.source;
    out.Timestamp = in.can_data.timestamp;
    return out;
  }
};
/*
struct xxxxx
{
  using cantype = gps::CAN;
  using cpptype = gps::xxxxx;
  using rostype = can_interfaces::msg::xxxxx;
  static constexpr const char * topic = "/gps/xxxxx";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out = in;
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out = static_cast<xxxx>(in);
    return out;
  }
};
*/
}
