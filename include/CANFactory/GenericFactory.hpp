///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <can_interfaces/msg/generic_msg.hpp>
#include "CanMessage.h"
#include <algorithm>
#include <cassert>

namespace sc
{
	struct SendGenericMsg
	{
	  static constexpr const char * topic = "/can/SendGeneric";
	};

	static inline can_interfaces::msg::GenericMsg MakeGeneric(const void* data, uint8_t length, uint16_t pgn, uint8_t bus, uint8_t priority, uint8_t destination = sc::NULL_ADDRESS)
	{
		can_interfaces::msg::GenericMsg msg;
		assert(length <= 8);
		if(length <= 8)
		{
			if(destination != sc::NULL_ADDRESS)
			{
				pgn = (pgn & 0xFF00) | destination;
			}
			msg.bus = bus;
			msg.can_data.frame_id = sc::MakeFrameId(pgn,priority);
			msg.can_data.timestamp = 0;
			memcpy(msg.data.data(), data, length);
			msg.length = length;
		}
		return msg;
	}
}