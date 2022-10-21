///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#pragma once
#include "CANTypes.h"
#include <JDMath/ParameterScaling.h>

namespace sc
{
   struct TwoBytePGNRequest
   {
      sc::pgn_t RequestedPGN;
      uint8_t Cmd1;
      uint8_t Cmd2;
   };

   struct CANTwoBytePGNRequest
   {
      enum { PGN = 0xEF00 };
      enum { CMD1 = 0xF1 };
      enum { CMD2 = 0x00 };
      enum { PRIORITY = 6 };

      uint16_t CmdBytes : 16;
      uint32_t RequestedPGN : 24;
      uint8_t Cmd1 : 8;
      uint8_t Cmd2 : 8;
      uint8_t Reserved1 : 8;

      inline TwoBytePGNRequest FromCAN(const sc::CANMsg& msg)
      {
         const CANTwoBytePGNRequest& in = sc::ToMsg<CANTwoBytePGNRequest>(msg);
         return TwoBytePGNRequest{
            .RequestedPGN =  static_cast<sc::pgn_t>(in.RequestedPGN),
            .Cmd1 = in.Cmd1,
            .Cmd2 = in.Cmd2
         };
      }

      inline CANTwoBytePGNRequest ToCAN(const TwoBytePGNRequest& in)
      {
         return CANTwoBytePGNRequest {
            .CmdBytes = cc::ToCMD16<CANTwoBytePGNRequest>(),
            .RequestedPGN = static_cast<uint32_t>(in.RequestedPGN),
            .Cmd1 = in.Cmd1,
            .Cmd2 = in.Cmd2,
            .Reserved1 = 0xFF
         };
      }
   }
   __attribute__((packed));
   ASSERT_EIGHT(CANTwoBytePGNRequest);
}
