///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <CANFactory/CanMessage.h>
#include <string>

namespace processing
{
	double ProcessLine(const std::string &line, sc::CanMessage_t& msg);
}

