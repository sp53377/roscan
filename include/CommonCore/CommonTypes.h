///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include <boost/geometry.hpp>
#include <boost/geometry/core/access.hpp>
#include <boost/geometry/core/coordinate_dimension.hpp>
#include <boost/geometry/core/coordinate_type.hpp>
#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <math.h>
#include <limits>
#include <string.h>

typedef float cm32_t;
typedef int32_t days32_t;
typedef days32_t days_t;
typedef float db32_t;
typedef float deg32_t;
typedef float celsius32_t;
typedef float deg_t;
typedef double deg64_t;
typedef float dpm32_t;
typedef float dpm_t;
typedef double dpm64_t;
typedef float dps32_t;
typedef double dps64_t;
typedef size_t hash_t;
typedef float ikm32_t;
typedef ikm32_t ikm_t;
typedef float km32_t;
typedef float kpa32_t;
typedef kpa32_t kpa_t;
typedef double kph64_t;
typedef float kph32_t;
typedef float kph_t;
typedef double lat64_t;
typedef double lon64_t;
typedef float m32_t;
typedef m32_t m_t;
typedef float sqm32_t;
typedef double sqm64_t;
typedef double m64_t;
typedef int32_t min32_t;
typedef min32_t min_t;
typedef float mm32_t;
typedef double mm64_t;
typedef float mph32_t;
typedef double mph64_t;
typedef float mps32_t;
typedef mps32_t mps_t;
typedef double mps64_t;
typedef float mpsSq32_t;
typedef mps32_t mpsSq_t;
typedef double mpsSq64_t;
typedef int32_t ms32_t;
typedef ms32_t ms_t;
typedef int64_t ms64_t;
typedef int32_t sec32_t;
typedef int32_t min32_t;
typedef min32_t min_t;
typedef float n32_t;
typedef n32_t n_t;
typedef float nm32_t;
typedef nm32_t nm_t;
typedef float pcnt32_t;
typedef pcnt32_t pcnt_t;
typedef float rad32_t;
typedef double rad64_t;
typedef float radPerM32_t;
typedef double radPerM64_t;
typedef float rpm32_t;
typedef rpm32_t rpm_t;
typedef uint16_t year16_t;
typedef year16_t year_t;
typedef uint8_t hz_t;
typedef float lpha32_t;
typedef float lpha_t;
typedef float v32_t;
typedef float v_t;
typedef double in64_t;
typedef float in32_t;

constexpr const hash_t INVALID_HASH = std::numeric_limits<hash_t>::max();
constexpr const size_t INVALID_SIZE = std::numeric_limits<size_t>::max();
constexpr const size_t INVALID_INDEX = std::numeric_limits<size_t>::max();
constexpr const deg64_t INVALID_TRACK_HEADING = std::numeric_limits<deg64_t>::max();
constexpr const int MBSD_INVALID_INDEX = -1;
constexpr const m64_t INVALID_DISTANCE = std::numeric_limits<m64_t>::max();
constexpr const m64_t MIN_DISTANCE = std::numeric_limits<m64_t>::min();


struct uuid_t
{
   enum {UUID_LENGTH = 39};//Includes room for braces and null termination
   char& operator[](int i) { return Id[i]; }
   char Id[UUID_LENGTH] = { 0 };
   static uuid_t FromString(const std::string& str)
   {
      uuid_t uuid;
      if(str.length() < UUID_LENGTH)
      {
         strncpy(uuid.Id, str.c_str(), UUID_LENGTH - 1);
         uuid.Id[UUID_LENGTH - 1] = '\0';
      }
      return uuid;
   }

};

enum EReceiverType
{
   RT_UNKNOWN,
   RT_VEHICLE,
   RT_IMPLEMENT
};

enum EActive
{
   AC_NotActive = 0,
   AC_Active = 1,
   AC_Error = 2,
   AC_NotAvailable = 3
};

enum EEnable
{
   EN_Disable = 0x00,
   EN_Enable = 0x01,
   EN_Error = 0x02,
   EN_NotAvailable = 0x03
};

enum ERequested
{
   RQ_NotRequested = 0x00,
   RQ_Requested = 0x01,
   RQ_Error = 0x02,
   RQ_NotAvailable = 0x03
};

enum ELocked
{
   LO_NotLocked = 0,
   LO_Locked = 1,
   LO_Error = 2,
   LO_NotAvaialble = 3
};

enum ECorrect
{
   CO_NotCorrect = 0,
   CO_Correct = 1,
   CO_Error = 2,
   CO_NotAvaialble = 3
};

enum EOnOff
{
   OO_Off = 0x00,
   OO_On = 0x01,
   OO_Error = 0x02,
   OO_NotAvailable = 0x03
};

enum EAllowed
{
   AL_NotAlowed = 0x00,
   AL_Allowed = 0x01,
   AL_Error = 0x02,
   AL_NotAvailable = 0x03
};

enum EReady
{
   RE_NotReady = 0x00,
   RE_Ready = 0x01,
   RE_Error = 0x02,
   RE_NotAvailable = 0x03
};

enum EEngaged
{
   EG_NotEngaged = 0x00,
   EG_Engaged = 0x01,
   EG_Error = 0x02,
   EG_NotAvailable = 0x03
};

enum EYesNo
{
   YN_No = 0x00,
   YN_Yes = 0x01,
   YN_Error = 0x02,
   YN_NotAvailable = 0x03
};

enum EAvailable
{
   AV_No = 0x00,
   AV_Yes = 0x01,
   AV_Error = 0x02,
   AV_NotAvailable = 0x03
};

enum EDirection //A.30.3
{
   D_Reverse = 0x00,
   D_Forward = 0x01,
   D_Error = 0x02,
   D_NotAvailable = 0x03
};

struct world_t
{
   lat64_t Lat = 0.0;
   lon64_t Lon = 0.0;
   m64_t Elevation = 0.0;
   static constexpr double TOLERANCE = 0.00000005; //~1mm

   bool isNear(const world_t& rhs, double tolerance = TOLERANCE) const
   {
      return (isEqual(Lat, rhs.Lat, tolerance)) &&
             (isEqual(Lon, rhs.Lon, tolerance)) &&
             (isEqual(Elevation, rhs.Elevation, tolerance));
   }

   bool isEqual(const double& lhs, const double& rhs, double epsilon = std::numeric_limits<double>::epsilon()) const
   {
      if(lhs == rhs)
      {
         return true;
      }
      return fabs(lhs - rhs) <= ((fabs(lhs) > fabs(rhs) ? fabs(rhs) : fabs(lhs)) * epsilon);
   }

   bool operator==(const world_t& rhs) const
   {
      return (isEqual(Lat, rhs.Lat)) &&
             (isEqual(Lon, rhs.Lon)) &&
             (isEqual(Elevation, rhs.Elevation));
   }

   bool operator!=(const world_t& rhs) const
   {
      return !this->operator==(rhs);
   }

   friend std::ostream& operator<<(std::ostream& out, const world_t& input)
   {
      out << "" << std::setprecision(12) << input.Lat <<
      ", " << std::setprecision(12) << input.Lon <<
      "";
      return out;
   }

};

static constexpr const world_t INVALID_WORLD_COORD{ 0, 0, 0 };

struct local_t
{
   m32_t N = 0.0f;
   m32_t E = 0.0f;
   m32_t Elevation = 0.0f;
   static constexpr double TOLERANCE = 0.001; //1mm

   bool isNear(const local_t& rhs, float tolerance = TOLERANCE) const
   {
      return (isEqual(N, rhs.N, tolerance)) &&
             (isEqual(E, rhs.E, tolerance)) &&
             (isEqual(Elevation, rhs.Elevation, tolerance));
   }

   bool isEqual(const float& lhs, const float& rhs, float epsilon = std::numeric_limits<float>::epsilon()) const
   {
      if(lhs == rhs)
      {
         return true;
      }
      return fabs(lhs - rhs) <= ((fabs(lhs) > fabs(rhs) ? fabs(rhs) : fabs(lhs)) * epsilon);
   }

   bool operator==(const local_t& rhs) const
   {
      return (isEqual(N, rhs.N)) &&
             (isEqual(E, rhs.E)) &&
             (isEqual(Elevation, rhs.Elevation));
   }

   bool operator!=(const local_t& rhs) const
   {
      return !this->operator==(rhs);
   }

   mm64_t CalculateDistanceTo(const local_t& local)
   {
      double dX = E - local.E;
      double dY = N - local.N;
      double dZ = Elevation - local.Elevation;
      double distanceBetween = sqrt((dX * dX) + (dY * dY));

      return sqrt((distanceBetween * distanceBetween) + (dZ * dZ));
   }

   friend std::ostream& operator<<(std::ostream& out, const local_t& input)
   {
      out << "{" <<
      "N " << input.N <<
      ", E " << input.E <<
      ", Elevation " << input.Elevation <<
      "}";
      return out;
   }

};

struct bounds_t
{
   m64_t X = 0.0;
   m64_t Y = 0.0;
   m64_t Z = 0.0;
   m64_t Width = 0.0;
   m64_t Length = 0.0;
   m64_t Height = 0.0;
};

struct BoundingCircle_t
{
   world_t Center;
   m32_t Radius;
};

struct Pose_t
{
   world_t WorldPos = { 0.0, 0.0, 0.0 };
   rad32_t Heading = { 0.0f };

   bool operator==(const Pose_t& rhs) const
   {
      return WorldPos == rhs.WorldPos && Heading == rhs.Heading;
   }

   bool operator!=(const Pose_t& rhs) const
   {
      return !(*this == rhs);
   }

   friend std::ostream& operator<<(std::ostream& out, const Pose_t& input)
   {
      out << "{" << std::setprecision(std::numeric_limits<double>::digits10) <<
      "Lat " << input.WorldPos.Lat <<
      ", Lon " << input.WorldPos.Lon <<
      ", Heading " << input.Heading <<
      "}";
      return out;
   }
};

struct LocalPose_t
{
   local_t Pos = { 0.0f, 0.0f, 0.0f };
   rad32_t Heading = { 0.0f };
};

static constexpr const Pose_t INVALID_POSE = Pose_t();

struct Transform_t
{
   world_t WorldPos;
   local_t LocalPos;
   rad32_t Bearing = 0.0f;
   deg32_t RawHeading = 0.0f;
   kph32_t Speed = 0.0f;
   bool IsValid = false;
   EDirection Direction = D_NotAvailable;

   friend std::ostream& operator<<(std::ostream& out, const Transform_t& input)
   {
      out << "{" <<
      "Bearing " << input.Bearing <<
      ", RawHeading " << input.RawHeading <<
      ", Speed " << input.Speed <<
      ", IsValid " << input.IsValid <<
      ", Direction " << (int)input.Direction <<
      "}";
      return out;
   }

};

enum {MAX_RESPONSE_TOPIC_LENGTH = 128};
template<typename T>
struct Request_t
{
   T Request;
   char ResponseTopic[MAX_RESPONSE_TOPIC_LENGTH] = "";
};

template<typename T>
static Request_t<T> CreateRequest(const std::string& responseTopic, const T& request)
{
   Request_t<T> req;
   req.Request = request;
   strncpy(req.ResponseTopic, responseTopic.c_str(), MAX_RESPONSE_TOPIC_LENGTH - 1);
   req.ResponseTopic[MAX_RESPONSE_TOPIC_LENGTH - 1] = '\0';
   return req;
}

template<typename T>
struct FastRequest_t
{
   T Request;
   hash_t ResponseTopic;
};

BOOST_GEOMETRY_REGISTER_POINT_2D(local_t, m32_t, cs::cartesian, N, E)
BOOST_GEOMETRY_REGISTER_POINT_2D(world_t, double, cs::cartesian, Lat, Lon)
