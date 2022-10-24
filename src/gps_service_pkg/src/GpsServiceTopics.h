///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <GpsService/GpsTypes.h>

namespace topics
{
   struct gps_GpsEpoch
   {
      static constexpr const char* Topic = "gps/GpsEpoch";
      gps::GpsEpoch Msg;
   };

   struct gps_GpsDevice
   {
      static constexpr const char* Topic = "gps/GpsDevice";
      gps::GpsDevice Msg;
   };

   struct gps_RequestGpsDevices
   {
      static constexpr const char* Topic = "gps/req/GpsDevices";
      hash_t Msg;
   };

   struct gps_GpsDisconnected
   {
      static constexpr const char* Topic = "gps/GpsDisconnected";
      gps::GpsDevice Msg;
   };

   struct gps_GpsEx
   {
      static constexpr const char* Topic = "gps/GpsEx";
      gps::GpsEx Msg;
   };

   struct gps_TimeDate
   {
      static constexpr const char* Topic = "gps/TimeDate";
      int64_t Timestamp;
   };

   struct gps_EpochReceived
   {
      static constexpr const char* Topic = "gps/EpochReceived";
      int64_t Timestamp;
   };

   struct gps_AllEpochsReceived
   {
      static constexpr const char* Topic = "gps/AllEpochsReceived";
      int64_t Timestamp;
   };

   struct gps_RepublishDatumRequest
   {
      static constexpr const char* Topic = "gps/req/RepublishDatum";
      //Empty
   };

   struct gps_DatumReceived
   {
      static constexpr const char* Topic = "gps/event/DatumReceived";
      gps::ReceiverDatum Datum;
   };

   struct gps_ExpectedDatum
   {
      static constexpr const char* Topic = "gps/event/ExpectedDatum";
      gps::ReceiverDatum Datum;
   };

   struct gps_DatumMatches
   {
      static constexpr const char* Topic = "gps/event/DatumMatches";
      gps::EDatumMatch Matches;
   };
}
