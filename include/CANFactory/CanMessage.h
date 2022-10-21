///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <iomanip>
#include <stdint.h>
#include <sstream>

namespace sc
{
struct CanMessage_t
{
  int8_t Channel = -1;
  uint32_t FrameId = 0;
  int8_t Length = -1;
  uint8_t Bytes[8];
};

struct MessageInstance_t
{
  double Timestamp = 0.0;
  sc::CanMessage_t Msg;
};

template<typename T>
CanMessage_t ToCanMessage(
  const typename T::cpptype & in, int8_t channel,
  int8_t destinationAddr = 0xFF)
{
  CanMessage_t msg;
  auto can = T::cantype::ToCAN(in);
  msg.Channel = channel;
  msg.Length = sizeof(typename T::cantype);
  memcpy(msg.Bytes, &can, msg.Length);
  msg.FrameId = (static_cast<uint32_t>(T::cantype::PRIORITY) << 26) |
    (static_cast<uint32_t>(T::cantype::PGN) << 8) | destinationAddr;
  return msg;
}

inline std::string ToString(const CanMessage_t & msg, const double & timestamp = 0.0)
{
  std::stringstream ss;
  ss << "ch:" << std::setfill('0') << std::setw(2) << (int) msg.Channel <<
    " pgn:" << std::setw(8) << std::hex << std::uppercase << msg.FrameId <<
    " len:" << std::setw(2) << (int) msg.Length;
  ss << " bytes:";
  for (int i = 0; i < 8; ++i) {
    if (i < msg.Length) {
      ss << " " << std::setfill('0') << std::setw(2) << std::hex << std::uppercase <<
      (int)msg.Bytes[i];
    } else {
      ss << "   ";
    }
  }
  ss << " ts:" << std::setprecision(12) << timestamp;
  return ss.str();
}

}
