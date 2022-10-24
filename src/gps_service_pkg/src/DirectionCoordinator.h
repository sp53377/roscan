///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <CommonCore/CommonTypes.h>

namespace gps
{
   class DirectionCoordinator
   {
   public:
      void UpdateCourseAndSpeed(deg32_t bearing, kph32_t speed, ms64_t timestamp);
      void UpdateNavBased(EDirection direction, bool isLowSpeedCapable, ms64_t timestamp);
      void UpdateTransmissionBasedDirection(EDirection direction, ms64_t timestamp);
      void UpdateWheelBasedDirection(EDirection direction, ms64_t timestamp);

      EDirection Get(int64_t timestamp);
   private:
      struct DirectionSource
      {
         ms64_t Timestamp {};
         EDirection Direction { D_NotAvailable };
      };

      void ToggleDirection();
      bool IsCourseInitialized() const;

      bool IsGpsValid { false };
      deg32_t PreviousCourse { -1 };
      DirectionSource NavBasedDirection;
      DirectionSource TransmissionBasedDirection;
      DirectionSource WheelBasedDirection;
      EDirection CourseBasedDirection { D_NotAvailable };
      EDirection SelectedDirection { D_NotAvailable };
      int NumberOfValidGpsCourseUpdates { 0 };
      kph32_t GPSMinTrustedSpeed { 0.5 };
   };
}
