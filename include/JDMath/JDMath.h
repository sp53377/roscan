///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <CommonCore/CommonTypes.h>
#include <limits>
#include <algorithm>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/arithmetic/arithmetic.hpp>
#include <boost/geometry/arithmetic/dot_product.hpp>

namespace bg = boost::geometry;

namespace jdm
{
   template<typename T>
   T pi() { return static_cast<T>(3.14159265358979); }
   template<typename T>
   T two_pi() { return static_cast<T>(3.14159265358979 * 2.0); }
   template<typename T>
   T half_pi() { return static_cast<T>(3.14159265358979 / 2.0); }

   template<typename T>
   T DegToRad(const T& deg)
   {
      return deg * pi<T>() / static_cast<T>(180.0);
   }

   template<typename T>
   T RadToDeg(const T& rad)
   {
      return rad * static_cast<T>(180.0) / pi<T>();
   }

   template<typename T>
   T MtoMM(const T& m)
   {
      return m * T(1000.0);
   }

   template<typename T>
   T MMtoM(const T& mm)
   {
      return mm / T(1000.0);
   }

   template<typename T>
   T KMtoM(const T& km)
   {
      return km * T(1000.0);
   }

   template<typename T>
   T MtoKM(const T& m)
   {
      return m / T(1000.0);
   }

   template<typename T>
   T MtoIN(const T& m)
   {
      return m * T(39.37007874);
   }

   template<typename T>
   T INtoM(const T& in)
   {
      return in / T(39.37007874);
   }

   template<typename T>
   T INtoMM(const T& in)
   {
      return in * T(1000.0) / T(39.37007874);
   }

   template<typename T>
   T MMtoIN(const T& mm)
   {
      return mm * T(39.37007874) / T(1000.0);
   }

   template<typename T>
   T FtToM(const T& ft)
   {
      return ft * T(0.3048);
   }

   template<typename T>
   T MtoFt(const T& m)
   {
      return m / T(0.3048);
   }

   template<typename T>
   T KphToMps(const T& kph)
   {
      return kph * static_cast<T>(0.277777777777778);
   }

   template<typename T>
   T MpsToKph(const T& mps)
   {
      return mps / static_cast<T>(0.277777777777778);
   }

   template<typename T>
   T KphToMph(const T& kph)
   {
      return kph * static_cast<T>(0.6213711922);
   }

   template<typename T>
   T MphToKph(const T& mph)
   {
      return mph / static_cast<T>(0.6213711922);
   }

   template<typename T>
   T MphToMps(const T& mph)
   {
      return mph * static_cast<T>(0.44704);
   }

   template<typename T>
   T MpsToMph(const T& mps)
   {
      return mps / static_cast<T>(0.44704);
   }

   //Degrees Per Meter to Radians per Kilometer
   template<typename T>
   T DPMToIKM(const T& dpm)
   {
      return DegToRad(dpm) * T(1000.0);
   }

   // Radians per Kilometer to Degrees Per Meter
   template<typename T>
   T RPKMToDPM(const T& ikm)
   {
      return RadToDeg(ikm) / T(1000.0);
   }

   template<typename T>
   T SqmToAc(const T& sqm)
   {
      return sqm * static_cast<T>(1.0 / 4046.86);
   }

   template<typename T>
   T AcToSqm(const T& ac)
   {
      return ac * static_cast<T>(4046.86);
   }

   template<typename T>
   typename bg::coordinate_type<T>::type Magnitude(const T& v)
   {
      auto x = bg::get<0>(v);
      auto y = bg::get<1>(v);
      return sqrt(x * x + y * y);
   }

   template<typename T>
   void SetToZeros(T& v)
   {
      bg::set<0>(v, 0);
      bg::set<1>(v, 0);
   }

   template<typename T>
   void Set(T& point, const typename bg::coordinate_type<T>::type& x, const typename bg::coordinate_type<T>::type& y)
   {
      bg::set<0>(point, x);
      bg::set<1>(point, y);
   }

   template<typename T>
   T MinCoord()
   {
      T result;
      bg::set<0>(result, std::numeric_limits<typename bg::coordinate_type<T>::type>::lowest());
      bg::set<1>(result, std::numeric_limits<typename bg::coordinate_type<T>::type>::lowest());
      return result;
   }

   template<typename T>
   T MaxCoord()
   {
      T result;
      bg::set<0>(result, std::numeric_limits<typename bg::coordinate_type<T>::type>::max());
      bg::set<1>(result, std::numeric_limits<typename bg::coordinate_type<T>::type>::max());
      return result;
   }

   template<typename T>
   T Min(const T& a, const T& b)
   {
      T result;
      bg::set<0>(result, std::min(bg::get<0>(a), bg::get<0>(b)));
      bg::set<1>(result, std::min(bg::get<1>(a), bg::get<1>(b)));
      return result;
   }

   template<typename T>
   T Max(const T& a, const T& b)
   {
      T result;
      bg::set<0>(result, std::max(bg::get<0>(a), bg::get<0>(b)));
      bg::set<1>(result, std::max(bg::get<1>(a), bg::get<1>(b)));
      return result;
   }

   template<typename T>
   double ToHeading(const T& v)
   {
      return atan2(bg::get<1>(v), bg::get<0>(v));
   }

   template<typename T>
   double ToHeading(const T& p1, const T& p2)
   {
      T v = p2;
      bg::subtract_point(v, p1);
      return ToHeading(v);
   }

   template<typename T>
   double Dx(const T& p1, const T& p2)
   {
      return bg::get<0>(p2) - bg::get<0>(p1);
   }

   template<typename T>
   double Dy(const T& p1, const T& p2)
   {
      return bg::get<1>(p2) - bg::get<1>(p1);
   }

   template<typename T>
   typename bg::coordinate_type<T>::type MagnitudeSquared(const T& v)
   {
      auto x = bg::get<0>(v);
      auto y = bg::get<1>(v);
      return x * x + y * y;
   }

   template<typename T>
   typename bg::coordinate_type<T>::type Manhattan(const T& a, const T& b)
   {
      auto dx = fabs(bg::get<0>(b) - bg::get<0>(a));
      auto dy = fabs(bg::get<1>(b) - bg::get<1>(a));
      return dx + dy;
   }

   template<typename T>
   T Clamp(const T& in, const T& min, const T& max)
   {
      return in < min ?
             min :
             (in > max ? max : in);
   }

   template<typename T>
   typename bg::coordinate_type<T>::type HeadingDelta(const T& headingVector, const T& p1, const T& p2)
   {
      T segmentVector = p2;
      bg::subtract_point(segmentVector, p1);
      double dot = bg::dot_product(headingVector, segmentVector);
      double length = jdm::Magnitude(segmentVector);
      return acos(jdm::Clamp(dot / length, -1.0, 1.0));
   }

   template<typename T>
   bool SameDirection(const T& v1, const T& v2)
   {
      return bg::dot_product(v1, v2) >= 0.0;
   }

   template<typename T>
   bool SameDirection(const T& v1, const T& p1, const T& p2)
   {
      T v2 = p2;
      bg::subtract_point(v2, p1);
      return bg::dot_product(v1, v2) >= 0.0;
   }

   template<typename T>
   typename bg::coordinate_type<T>::type HeadingDelta(const T& heading, const T& p1, const T& p2, bool sameDirection)
   {
      return sameDirection ?
             HeadingDelta(heading, p1, p2) :
             HeadingDelta(heading, p2, p1);
   }

   template<typename T>
   bool ProjectsOntoSegment(const T& point, const T& a, const T& b, const double tolerance = T::TOLERANCE)
   {
      bool result = false;
      T abVec = b;
      bg::subtract_point(abVec, a);
      T acVec = point;
      bg::subtract_point(acVec, a);
      double abMagnitude = jdm::Magnitude(abVec);
      if(abMagnitude > 0.0)
      {
         double distance = bg::dot_product(abVec, acVec) / abMagnitude;
         if(distance >= -tolerance && distance <= abMagnitude + tolerance)
         {
            result = true;
         }
      }
      return result;
   }

   template<typename T>
   typename bg::coordinate_type<T>::type Distance(const T& a, const T& b)
   {
      return bg::distance(a, b);
   }

   template<typename T>
   T PointOnSegment(const T& a, const T& b, const typename bg::detail::param<T>::type& percentAlongSegment)
   {
      T abVec = b;
      bg::subtract_point(abVec, a);
      bg::multiply_value(abVec, percentAlongSegment);
      bg::add_point(abVec, a);
      return abVec;
   }

   template<typename T>
   T Unit(const T& v)
   {
      T unitVec = v;
      bg::divide_value(unitVec, Magnitude(v));
      return unitVec;
   }

   template<typename T>
   T Normal(const T& v)
   {
      T result;
      bg::set<0>(result, -bg::get<1>(v));
      bg::set<1>(result, bg::get<0>(v));
      return result;
   }

   template<typename T>
   typename bg::coordinate_type<T>::type DistanceSq(const T& a, const T& b)
   {
      auto dx = bg::get<0>(b) - bg::get<0>(a);
      auto dy = bg::get<1>(b) - bg::get<1>(a);
      return dx * dx + dy * dy;
   }

   template<typename T>
   T Subtract(const T& a, const T& b)
   {
      T result = a;
      bg::subtract_point(result, b);
      return result;
   }

   template<typename T>
   T Add(const T& a, const T& b)
   {
      T result = a;
      bg::add_point(result, b);
      return result;
   }

   template<typename T>
   T Multiply(T a, const double& scale)
   {
      bg::multiply_value(a, scale);
      return a;
   }

   template<typename T>
   typename bg::coordinate_type<T>::type PerpendicularProduct(const T& a, const T& b)
   {
      //return ax * by - ay * bx;
      return bg::get<0>(a) * bg::get<1>(b) - bg::get<1>(a) * bg::get<0>(b);
   }

   template<typename T>
   T NormalizeAngle(const T& rad)
   {
      T result = fmod(rad, jdm::two_pi<T>());
      if(result > jdm::pi<T>())
      {
         result -= jdm::two_pi<T>();
      }
      else if(result < -jdm::pi<T>())
      {
         result += jdm::two_pi<T>();
      }
      return result;
   }

   template<typename T>
   T NormalizedHeading(const T& heading)
   {
      T normalizedHeading = fmod(heading, jdm::two_pi<T>());

      if(normalizedHeading < 0)
      {
         normalizedHeading += jdm::two_pi<T>();
      }

      return normalizedHeading;
   }

   template<typename T>
   T NormalizedHeadingDiff(T delta)
   {
      while(delta > jdm::pi<T>())
      {
         delta -= jdm::two_pi<T>();
      }

      while(delta < -jdm::pi<T>())
      {
         delta += jdm::two_pi<T>();
      }

      return delta;
   }

   template<typename T>
   T LimitHeadingDiff(T delta)
   {
      if(delta > jdm::half_pi<T>())
      {
         delta = jdm::half_pi<T>();
      }
      else if(delta < -jdm::half_pi<T>())
      {
         delta = -jdm::half_pi<T>();
      }

      return delta;
   }

   template<typename T>
   T AngleDeltaRad(const T& rad1, const T& rad2)
   {
      return NormalizedHeadingDiff(rad2 - rad1);
   }

   template<typename T>
   T GetCentroid(const std::vector<T>& points)
   {
      T centroid;
      if(points.size() > 0)
      {
         centroid = points[0];
         for(size_t i = 1; i < points.size(); ++i)
         {
            bg::add_point(centroid, points[i]);
         }

         bg::divide_value(centroid, points.size());
      }
      else
      {
         SetToZeros(centroid);
      }
      return centroid;
   }

   template<typename T>
   bool Equals(const T& a, const T& b)
   {
      return bg::equals(a, b);
   }

   template<typename T>
   bool NotEqual(const T& a, const T& b)
   {
      return !bg::equals(a, b);
   }

   template<typename T>
   typename bg::coordinate_type<T>::type InlineDistance(const T& a, const T& b, const T& point, const m64_t& length)
   {
      typename bg::coordinate_type<T>::type inlineDistance = std::numeric_limits<float>::max();
      if(length > 0.0)
      {
         T abVec = Subtract(b, a);
         T acVec = Subtract(point, a);

         inlineDistance =  bg::dot_product(abVec, acVec) / length;
      }

      return inlineDistance;
   }

   template<typename T>
   typename bg::coordinate_type<T>::type InlineDistance(const T& a, const T& b, const T& point)
   {
      return InlineDistance(a, b, point, Distance(a, b));
   }

   template<typename T>
   typename bg::coordinate_type<T>::type LateralDistance(const T& a, const T& b, const T& point, const m64_t& length)
   {
      typename bg::coordinate_type<T>::type lateralDistance = std::numeric_limits<float>::max();
      if(length > 0.0)
      {
         T abVec = Subtract(b, a);
         T acVec = Subtract(point, a);

         lateralDistance = -PerpendicularProduct(abVec, acVec) / length;
      }

      return lateralDistance;
   }

   template<typename T>
   typename bg::coordinate_type<T>::type LateralDistance(const T& a, const T& b, const T& point)
   {
      return LateralDistance(a, b, point, Distance(a, b));
   }

   template<typename T>
   bool IsSegmentRelevant(const T& a, const T& b, const T& position, const rad64_t& previousHeading, const rad64_t& nextHeading, const double tolerance = T::TOLERANCE)
   {
      m64_t length = jdm::Distance(a, b);
      m64_t inlineDistance = jdm::InlineDistance(a, b, position);
      m64_t lateralDistance = jdm::LateralDistance(a, b, position);
      rad64_t currentHeading = jdm::ToHeading(a, b);

      rad64_t previousHeadingDelta = LimitHeadingDiff(NormalizedHeadingDiff(currentHeading - previousHeading));
      m64_t previousInlineExtension = lateralDistance * tan(previousHeadingDelta / 2.0);

      rad64_t nextHeadingDelta =  LimitHeadingDiff(NormalizedHeadingDiff(nextHeading - currentHeading));
      m64_t nextInlineExtension = lateralDistance * tan(nextHeadingDelta / 2.0);

      m64_t lowerLimit = -(previousInlineExtension + tolerance);
      m64_t upperLimit = length + nextInlineExtension + tolerance;
      return inlineDistance >= lowerLimit && inlineDistance <= upperLimit;
   }

   template<typename T>
   bool IsSegmentInlined(const T& a, const T& b, const T& position, const double tolerance = T::TOLERANCE)
   {
      m64_t length = jdm::Distance(a, b);
      m64_t inlineDistance = jdm::InlineDistance(a, b, position);

      return inlineDistance >= (0 - tolerance) && inlineDistance <= (length + tolerance);
   }

   template<typename T>
   bool NearestPointOnLine(const T& a, const T& b, const T& point, T& result)
   {
      bool success = false;
      double segmentLength = jdm::Distance(a, b);
      if(segmentLength > 0.0)
      {
         double inlineDistance = jdm::InlineDistance(a, b, point);
         double ratioAlongSegment = inlineDistance / segmentLength;
         result = PointOnSegment(a, b, ratioAlongSegment);
         success = true;
      }
      return success;
   }

   template<typename T>
   bool NearestPointOnSegment(const T& a, const T& b, const T& point, T& result)
   {
      bool success = false;
      double segmentLength = jdm::Distance(a, b);
      if(segmentLength > 0.0)
      {
         double inlineDistance = jdm::InlineDistance(a, b, point);
         inlineDistance = std::max(inlineDistance, 0.0);
         inlineDistance = std::min(inlineDistance, segmentLength);
         double ratioAlongSegment = inlineDistance / segmentLength;
         result = PointOnSegment(a, b, ratioAlongSegment);
         success = true;
      }
      return success;
   }

   template<typename T>
   bool NearestPointActuallyOnSegment(const T& a, const T& b, const T& point, T& result, m64_t* inlineDistanceOut = nullptr, const double tolerance = T::TOLERANCE)
   {
      bool success = false;
      double segmentLength = jdm::Distance(a, b);
      if(segmentLength > 0.0)
      {
         double inlineDistance = jdm::InlineDistance(a, b, point);
         if(inlineDistanceOut)
         {
            *inlineDistanceOut = inlineDistance;
         }

         if(inlineDistance >= (0.0 - tolerance) && inlineDistance <= (segmentLength + tolerance))
         {
            double ratioAlongSegment = inlineDistance / segmentLength;
            result = PointOnSegment(a, b, ratioAlongSegment);
            success = true;
         }
      }
      return success;
   }

   //Return Distance:
   template<typename P, typename S>
   double DistanceBetweenSegments(const S& a, const S& b, P& outPointA, P& outPointB)
   {
      double distance = 0.0;
      std::vector<P> intersections;
      rad64_t angleA = ToHeading(a.first, a.second);
      rad64_t angleB = ToHeading(b.first, b.second);
      constexpr static const m64_t TOLERANCE = 1e-6;
      bool collinear = fabs(NormalizedHeadingDiff(angleA - angleB)) < TOLERANCE;
      // boost intersection return worng intersection that is out of range if 2 segment is collinear
      if(!collinear &&  bg::intersection(a, b, intersections) && intersections.size() == 1)
      {
         outPointA = intersections.front();
         outPointB = outPointA;
      }
      else
      {
         const P& a1 = a.first;
         const P& a2 = a.second;
         const P& b1 = b.first;
         const P& b2 = b.second;
         P ra1;
         P ra2;
         P rb1;
         P rb2;
         if(NearestPointOnSegment(a1, a2, b1, ra1) &&
            NearestPointOnSegment(a1, a2, b2, ra2) &&
            NearestPointOnSegment(b1, b2, a1, rb1) &&
            NearestPointOnSegment(b1, b2, a2, rb2)
            )
         {
            double d1 = bg::distance(ra1, b1);
            double d2 = bg::distance(ra2, b2);
            double d3 = bg::distance(rb1, a1);
            double d4 = bg::distance(rb2, a2);

            if(d1 <= d2 && d1 <= d3 && d1 <= d4)
            {
               outPointA = ra1;
               outPointB = b1;
               distance = d1;
            }
            else if(d2 <= d1 && d2 <= d3 && d2 <= d4)
            {
               outPointA = ra2;
               outPointB = b2;
               distance = d2;
            }
            else if(d3 <= d1 && d3 <= d2 && d3 <= d4)
            {
               outPointA = a1;
               outPointB = rb1;
               distance = d3;
            }
            else if(d4 <= d1 && d4 <= d2 && d4 <= d3)
            {
               outPointA = a2;
               outPointB = rb2;
               distance = d4;
            }

         }
         else
         {
            //Segment Length is 0.0

         }
      }
      return distance;
   }

   template<typename T>
   bool EqZero(const T& num, const T& tolerance)
   {
      return (fabs(num) < fabs(tolerance));
   }

   template<typename T>
   bool Eq(const T& lhs, const T& rhs, const T& tolerance)
   {
      return EqZero(lhs - rhs, tolerance);

   }

   template<typename T>
   bool LessThan(const T& lhs, const T& rhs, const T& tolerance)
   {
      return ((lhs < rhs) && !Eq(lhs, rhs, tolerance));
   }

   template<typename T>
   bool LessThanOrEqual(const T& lhs, const T& rhs, const T& tolerance)
   {
      return ((lhs < rhs) || Eq(lhs, rhs, tolerance));
   }

   template<typename T>
   bool GreaterThan(const T& lhs, const T& rhs, const T& tolerance)
   {
      return ((lhs > rhs) && !Eq(lhs, rhs, tolerance));
   }

   template<typename T>
   bool GreaterThanOrEqual(const T& lhs, const T& rhs, const T& tolerance)
   {
      return ((lhs > rhs) || Eq(lhs, rhs, tolerance));
   }

   template<typename T>
   T Interpolate(const T& a, const T& b, const double& ratio)
   {
      T result = Subtract(b, a);
      result = Multiply(result, ratio);
      result = Add(a, result);
      return result;
   }

}
