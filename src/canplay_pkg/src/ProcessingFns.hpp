#pragma once

#include <CANFactory/CanMessage.h>
#include <string>

namespace processing
{
	double ProcessLine(const std::string &line, sc::CanMessage_t& msg);
}

