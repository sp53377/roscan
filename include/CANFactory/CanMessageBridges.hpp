///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <CANFactory/PGNRequest.h>
#include <can_interfaces/msg/pgn_request.hpp>

namespace bridge
{
struct PgnRequestBase
{
  using cantype = sc::CANPGNRequest;
  using cpptype = sc::PGNRequest;
  using rostype = can_interfaces::msg::PgnRequest;

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out.pgn = in.RequestedPGN;
    out.destination_addr = in.Destination;
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    sc::CanDataFromROS(in, pSource, pTimestamp);
    out.RequestedPGN = in.pgn;
    out.Destination = in.destination_addr;
    return out;
  }
};

struct GlobalPgnRequest : public PgnRequestBase
{
  static constexpr const char * topic = "/sc/GlobalPgnRequest";
};

struct PgnRequest : public PgnRequestBase
{
  static constexpr const char * topic = "/sc/PgnRequest";
};
/*
struct xxxxx
{
  using cantype = gps::CAN;
  using cpptype = gps::xxxxx;
  using rostype = can_interfaces::msg::xxxxx;
  static constexpr const char * topic = "/gps/xxxxx";

  static inline rostype ToROS(const cpptype & in)
  {
    rostype out;
    out = in;
    return out;
  }

  static inline cpptype FromROS(const rostype & in, uint8_t* pSource= nullptr, int64_t* pTimestamp= nullptr)
  {
    cpptype out;
    bridge::CanDataFromROS(in, pSource, pTimestamp);
    out = static_cast<xxxx>(in);
    return out;
  }
};
*/
}
