///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <CANFactory/CanMessage.h>

struct MessageInstance_t
{
  double Timestamp = 0.0;
  sc::CanMessage_t Msg;
};

class ICanSource
{
  public:
    virtual ~ICanSource(){};
    virtual bool Step(bool& isValidOut, MessageInstance_t& messageOut) = 0;
};