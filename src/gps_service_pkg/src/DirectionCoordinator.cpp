///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#include "DirectionCoordinator.h"

using namespace gps;

namespace
{
   constexpr int NUMBER_OF_EPOCHS_FOR_INITILIZATION = 5;
   constexpr deg32_t DIRECTION_TOGGLE_MIN_ANGLE = 90;
   constexpr ms32_t NAV_BASED_SETTLE_TIME = 2000;
   constexpr ms32_t TRANSMISSION_BASED_SETTLE_TIME = 3000;
   constexpr ms32_t WHEEL_BASED_SETTLE_TIME = 3000;

   deg32_t NormalizedDiff(const deg32_t prevBearing, const deg32_t currentBearing)
   {
      deg32_t diff = fabs(prevBearing - currentBearing);
      return (diff > 180.0) ? 360 - diff : diff;
   }

   bool IsValid(const EDirection direction)
   {
      return direction < D_Error;
   }
}

void DirectionCoordinator::UpdateCourseAndSpeed(const deg32_t bearing, const kph32_t speed, const ms64_t /*timestamp*/)
{
   if(bearing >= 0 && bearing <= 360)
   {
      const deg32_t prevBearing = (PreviousCourse < 0.0f) ? bearing : PreviousCourse;
      const deg32_t diff = NormalizedDiff(prevBearing, bearing);
      const bool directionChanged = diff > DIRECTION_TOGGLE_MIN_ANGLE;
      const bool isGpsValid = speed > GPSMinTrustedSpeed;
      if(NumberOfValidGpsCourseUpdates < NUMBER_OF_EPOCHS_FOR_INITILIZATION)
      {
         CourseBasedDirection = SelectedDirection;
         if(not IsValid(CourseBasedDirection) && IsValid(NavBasedDirection.Direction))
         {
            CourseBasedDirection = NavBasedDirection.Direction;
         }
         if(IsValid(CourseBasedDirection) && isGpsValid && not directionChanged)
         {
            ++NumberOfValidGpsCourseUpdates;
         }
         else
         {
            NumberOfValidGpsCourseUpdates = 0;
         }
      }
      else
      {
         if(isGpsValid && directionChanged)
         {
            ToggleDirection();
         }
         IsGpsValid = isGpsValid;
      }
      if(isGpsValid)
      {
         PreviousCourse = bearing;
      }
   }
}

void DirectionCoordinator::UpdateNavBased(const EDirection direction, const bool isLowSpeedCapable, const ms64_t timestamp)
{
   if(direction != NavBasedDirection.Direction)
   {
      NavBasedDirection = { timestamp, direction };
   }
   GPSMinTrustedSpeed = isLowSpeedCapable ? 0.1 : 0.5;
}

void DirectionCoordinator::UpdateTransmissionBasedDirection(const EDirection direction, const ms64_t timestamp)
{
   if(direction != TransmissionBasedDirection.Direction)
   {
      TransmissionBasedDirection = { timestamp, direction };
   }
}

EDirection DirectionCoordinator::Get(ms64_t timestamp)
{
   auto resetInitOnDirectionChange = [this](EDirection direction) {
      if(NumberOfValidGpsCourseUpdates < NUMBER_OF_EPOCHS_FOR_INITILIZATION && SelectedDirection != direction)
      {
         NumberOfValidGpsCourseUpdates = 0;
      }
   };
   if(IsGpsValid)
   {
      if(IsValid(NavBasedDirection.Direction) &&
            SelectedDirection != NavBasedDirection.Direction &&
            timestamp - NavBasedDirection.Timestamp >= NAV_BASED_SETTLE_TIME)
      {
         //EVL_CONSOLE() << "Course change missed. Sync with Nav based direction: " << NavBasedDirection.Direction;
         SelectedDirection = NavBasedDirection.Direction;
      }
      else
      {
         SelectedDirection = CourseBasedDirection;
      }
   }
   else
   {
      if(IsValid(TransmissionBasedDirection.Direction) &&
            (timestamp - TransmissionBasedDirection.Timestamp >= TRANSMISSION_BASED_SETTLE_TIME or !IsCourseInitialized()))
      {
         resetInitOnDirectionChange(TransmissionBasedDirection.Direction);
         SelectedDirection = TransmissionBasedDirection.Direction;
      }
      else if(IsValid(WheelBasedDirection.Direction) &&
              (timestamp - WheelBasedDirection.Timestamp >= WHEEL_BASED_SETTLE_TIME or !IsCourseInitialized()))
      {
         resetInitOnDirectionChange(WheelBasedDirection.Direction);
         SelectedDirection = WheelBasedDirection.Direction;
      }
   }
   return SelectedDirection;
}

void DirectionCoordinator::UpdateWheelBasedDirection(const EDirection direction, const ms64_t timestamp)
{
   WheelBasedDirection = { timestamp, direction };
}

void DirectionCoordinator::ToggleDirection()
{
   NavBasedDirection.Direction = D_NotAvailable;
   WheelBasedDirection.Direction = D_NotAvailable;
   TransmissionBasedDirection.Direction = D_NotAvailable;
   CourseBasedDirection = CourseBasedDirection == D_Reverse ? D_Forward : D_Reverse;
}

bool DirectionCoordinator::IsCourseInitialized() const
{
   return NumberOfValidGpsCourseUpdates >= NUMBER_OF_EPOCHS_FOR_INITILIZATION;
}
