///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#pragma once
#include <math.h>

namespace jdm
{

   template <typename T>
   struct CircleIntersection_t
   {
      T x1 = 0;
      T y1 = 0;
      T x2 = 0;
      T y2 = 0;
      bool valid = false;
   };

   template <typename T>
   struct JDCircle
   {
   public:
      T X = 0;
      T Y = 0;
      T Radius = 0;

      JDCircle(const T& cx,const T& cy, const T& radius)
         :X(cx)
         ,Y(cy)
         ,Radius(radius)
      {
      }

      CircleIntersection_t<T> Nearest(const JDCircle<T>& c2) const
      {
         CircleIntersection_t<T> result;
         auto dx = c2.X - X;
         auto dy = c2.Y - Y;
         auto d = sqrt(dx*dx+dy*dy);
         bool coincident = d == 0;
         if(!coincident)
         {
            auto ux = dx/d;
            auto uy = dy/d;
            result.x1 = X + Radius * ux;
            result.y1 = Y + Radius * uy;
            result.x2 = c2.X - c2.Radius * ux;
            result.y2 = c2.Y - c2.Radius * uy;
            result.valid = true;
         }
         return result;
      }

      CircleIntersection_t<T> Intersection(const JDCircle<T>& c2) const
      {
         CircleIntersection_t<T> result;
         auto dx = c2.X - X;
         auto dy = c2.Y - Y;
         auto d = sqrt(dx*dx+dy*dy);
         bool outside = d > Radius + c2.Radius;
         bool inside = d < fabs(Radius - c2.Radius);
         bool coincident = d == 0 && Radius == c2.Radius;
         if(!outside && !inside && !coincident)
         {
            auto r1Sq = Radius * Radius;
            auto r2Sq = c2.Radius * c2.Radius;
            auto dSq = d * d;
            auto a = (r1Sq - r2Sq + dSq) / (2 * d);
            auto aSq = a * a;
            auto h = sqrt(r1Sq - aSq);
            auto x2 = X + a * dx / d;
            auto y2 = Y + a * dy / d;
            auto vx = h * dy / d;
            auto vy = -h * dx / d;
            result.x1 = x2 + vx;
            result.y1 = y2 + vy;
            result.x2 = x2 - vx;
            result.y2 = y2 - vy;
            result.valid = true;
         }
         return result;
      }
   };
}
