#pragma once
#include <CANFactory/CanMessage.h>

class ICanSource
{
  public:
    virtual ~ICanSource(){};
    virtual bool Step(bool& isValidOut, sc::MessageInstance_t& messageOut) = 0;
};