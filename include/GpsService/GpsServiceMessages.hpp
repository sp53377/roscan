#pragma once

#include <GpsService/GpsMessages.h>
#include <can_interfaces/msg/bearing_speed.hpp>
#include <can_interfaces/msg/vehicle_position.hpp>

namespace bridge
{
	struct BearingSpeed
	{
		using cantype = gps::CANBearingSpeed;
		using cpptype = gps::BearingSpeed;
		using rostype = can_interfaces::msg::BearingSpeed;

		static inline rostype ToROS(const cpptype& in)
		{
			rostype out;
			out.bearing_deg = in.Bearing;
			out.speed_kph = in.Speed;
			return out;
		}

		static inline cpptype FromROS(const rostype& in)
		{
			cpptype out;
			out.Bearing = in.bearing_deg;
			out.Speed = in.speed_kph;
			return out;
		}
	};

	struct VehiclePosition
	{
		using cantype = gps::CANVehiclePosition;
		using cpptype = gps::VehiclePosition;
		using rostype = can_interfaces::msg::VehiclePosition;

		static inline rostype ToROS(const cpptype& in)
		{
			rostype out;
			out.latitude = in.Latitude;
			out.longitude = in.Longitude;
			return out;
		}

		static inline cpptype FromROS(const rostype& in)
		{
			cpptype out;
			out.Latitude = in.latitude;
			out.Longitude = in.longitude;
			return out;
		}
	};
}