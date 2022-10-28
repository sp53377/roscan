///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#pragma once
#include "CANTypes.h"

namespace sc
{
   struct PGNRequest
   {
      sc::pgn_t RequestedPGN = 0xFF;
      sc::addr_t Destination = 0xFF;
   };

   struct CANPGNRequest
   {
      enum { PGN = 0xEA00 };
      enum { PRIORITY = 6 };

      uint8_t RequestedPGN[3];

      static inline PGNRequest FromCAN(const sc::CANMsg& msg)
      {
         const CANPGNRequest& in = sc::ToMsg<CANPGNRequest>(msg);
         PGNRequest out;
	 out.RequestedPGN = (static_cast<uint32_t>(in.RequestedPGN[2]) << 16) | (static_cast<uint32_t>(in.RequestedPGN[1]) << 16) | (static_cast<uint32_t>(in.RequestedPGN[0]));
	 out.Destination = msg.Destination;
         return out;
      }

      static inline CANPGNRequest ToCAN(const PGNRequest& in)
      {
         CANPGNRequest out;
         out.RequestedPGN[0] = static_cast<uint8_t>(in.RequestedPGN & 0x0000FF);
         out.RequestedPGN[1] = static_cast<uint8_t>((in.RequestedPGN & 0x00FF00) >> 8);
         out.RequestedPGN[2] = static_cast<uint8_t>((in.RequestedPGN & 0xFF0000) >> 16);
         return out;
      }
   };
   ASSERT_SIZE(CANPGNRequest,3);
}
