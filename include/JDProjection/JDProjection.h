///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <CommonCore/CommonTypes.h>
#include <math.h>

namespace jdp
{
   class JDProjection
   {
   public:
      JDProjection()
         : ReferenceLatitude(0.0)
         , ReferenceLongitude(0.0)
         , NorthingsPerDegree(1.0)
         , EastingsPerDegree(1.0)
      {
      }

      JDProjection(const JDProjection& rhs)
         : ReferenceLatitude(rhs.ReferenceLatitude)
         , ReferenceLongitude(rhs.ReferenceLongitude)
         , NorthingsPerDegree(rhs.NorthingsPerDegree)
         , EastingsPerDegree(rhs.EastingsPerDegree)
      {
      }

      JDProjection(const lat64_t& latitude, const lon64_t longitude)
         : ReferenceLatitude(latitude)
         , ReferenceLongitude(longitude)
         , NorthingsPerDegree(1.0)
         , EastingsPerDegree(1.0)
      {
         RecomputeScales();
      }

      JDProjection(const world_t& referencePt)
         : ReferenceLatitude(referencePt.Lat)
         , ReferenceLongitude(referencePt.Lon)
         , NorthingsPerDegree(1.0)
         , EastingsPerDegree(1.0)
      {
         RecomputeScales();
      }

      JDProjection& operator=(const JDProjection& rhs)
      {
         ReferenceLatitude = rhs.ReferenceLatitude;
         ReferenceLongitude = rhs.ReferenceLongitude;
         NorthingsPerDegree = rhs.NorthingsPerDegree;
         EastingsPerDegree = rhs.EastingsPerDegree;
         return *this;
      }

      bool IsValid() const
      {
         return ReferenceLatitude != 0.0 && ReferenceLongitude != 0.0;
      }

      void Clear()
      {
         ReferenceLatitude = 0.0;
         ReferenceLongitude = 0.0;
      }

      void SetReference(const lat64_t& latitude, const lon64_t& longitude)
      {
         ReferenceLatitude = latitude;
         ReferenceLongitude = longitude;
         RecomputeScales();
      }

      void SetReference(const world_t& referencePoint)
      {
         SetReference(referencePoint.Lat, referencePoint.Lon);
      }

      lat64_t ReferenceLat() const
      {
         return ReferenceLatitude;
      }

      lon64_t ReferenceLon() const
      {
         return ReferenceLongitude;
      }

      world_t ReferencePoint() const
      {
         return world_t{ ReferenceLatitude, ReferenceLongitude, 0.0 };
      }

      m64_t NorthingsPerDeg() const
      {
         return NorthingsPerDegree;
      }

      m64_t EastingsPerDeg() const
      {
         return EastingsPerDegree;
      }

      m64_t ToNorthings(const lat64_t& latitude) const
      {
         return NorthingsPerDegree * (latitude - ReferenceLatitude);
      }

      m64_t ToEastings(const lon64_t& longitude) const
      {
         return EastingsPerDegree * (longitude - ReferenceLongitude);
      }

      local_t ToLocal(const world_t& world) const
      {
         return local_t{
            m32_t(NorthingsPerDegree * (world.Lat - ReferenceLatitude)),
            m32_t(EastingsPerDegree * (world.Lon - ReferenceLongitude)),
            m32_t(world.Elevation)
         };
      }

      lat64_t ToLatitude(const m64_t& northings) const
      {
         return (northings / NorthingsPerDegree) + ReferenceLatitude;
      }

      lon64_t ToLongitude(const m64_t& eastings) const
      {
         return (eastings / EastingsPerDegree) + ReferenceLongitude;
      }

      lat64_t ToLatitudeOffset(const m64_t& northings) const
      {
         return (northings / NorthingsPerDegree);
      }

      lon64_t ToLongitudeOffset(const m64_t& eastings) const
      {
         return (eastings / EastingsPerDegree);
      }

      world_t ToWorld(const local_t& local) const
      {
         return world_t{
            (local.N / NorthingsPerDegree) + ReferenceLatitude,
            (local.E / EastingsPerDegree) + ReferenceLongitude,
            local.Elevation
         };
      }

      world_t ToWorld(const m64_t& northing, const m64_t& easting, const m64_t& elevation = 0.0) const
      {
         return world_t{
            (northing / NorthingsPerDegree) + ReferenceLatitude,
            (easting / EastingsPerDegree) + ReferenceLongitude,
            elevation
         };
      }

      lat64_t LatitudeFrom(const lat64_t& latitude, const m64_t& northingsFrom) const
      {
         return (northingsFrom / NorthingsPerDegree) + latitude;
      }

      lon64_t LongitudeFrom(const lon64_t& longitude, const m64_t& eastingFrom) const
      {
         return (eastingFrom / EastingsPerDegree) + longitude;
      }

      m64_t Distance(const lat64_t& lat1, const lon64_t& lon1,
            const lat64_t& lat2, const lon64_t& lon2) const
      {
         return sqrt(DistanceSq(lat1, lon1, lat2, lon2));
      }

      m64_t Distance(const world_t& a, const world_t& b) const
      {
         return sqrt(DistanceSq(a.Lat, a.Lon, b.Lat, b.Lon));
      }

      m64_t DistanceSq(const lat64_t& lat1, const lon64_t& lon1,
            const lat64_t& lat2, const lon64_t& lon2) const
      {
         double a = (lat2 - lat1) * NorthingsPerDegree;
         double b = (lon2 - lon1) * EastingsPerDegree;
         return a * a + b * b;
      }

      m64_t DistanceSq(const world_t& a, const world_t& b) const
      {
         return DistanceSq(a.Lat, a.Lon, b.Lat, b.Lon);
      }

   private:
      void RecomputeScales()
      {
         static const double TO_RAD = 3.14159265358979 / 180.0;
         static const double REF_ELLIPSOID_A = 6378137.0e0; // meter
         // f = 1/298.257223563
         // e^2 = 2*f - f*f eccentricity of the ellipsoid
         static const double REF_ELLIPSOID_ECCENTRICITY = 0.006694379990141317;
         static const double METERS_PER_DEGREE_CONVERSION_FACTOR = 0.01745329251994;

         double sinlat, nu, dtmp;

         double radLat(ReferenceLatitude * TO_RAD);

         sinlat = sin(radLat);

         /* Compute radius of curvature in prime meridian */
         dtmp = sqrt(1.0 - (REF_ELLIPSOID_ECCENTRICITY * sinlat * sinlat));

         if(fabs(sinlat) < 0.0001)
         {
            // close to the equator
            nu = REF_ELLIPSOID_A * (1.0 + (REF_ELLIPSOID_ECCENTRICITY * sinlat * sinlat * 0.5));
         }
         else
         {
            // far from the equator
            nu = REF_ELLIPSOID_A / dtmp;
         }

         /* compute and save scale factors for converting short distance north and
               east in meters to lat and lon in radians
               BEFORE multiplying the conversion factor*/

         /* rho */
         NorthingsPerDegree = (REF_ELLIPSOID_A * (1e0 - REF_ELLIPSOID_ECCENTRICITY) / (dtmp * dtmp * dtmp)) *
                              METERS_PER_DEGREE_CONVERSION_FACTOR;

         /* nu*coslat */
         EastingsPerDegree = nu * cos(radLat) * METERS_PER_DEGREE_CONVERSION_FACTOR;
      }

      lat64_t ReferenceLatitude;
      lon64_t ReferenceLongitude;

      m64_t NorthingsPerDegree;
      m64_t EastingsPerDegree;
   };
}
