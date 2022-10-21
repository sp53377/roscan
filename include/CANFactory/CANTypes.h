///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <functional>
#include <cstddef>
#include <list>
#include <map>
#include <memory.h>
#include <type_traits>
#include <vector>

namespace sc
{
   typedef uint8_t addr_t;
   typedef uint16_t node_t;
   typedef uint32_t pgn_t;
   typedef uint8_t bus_t;

   struct IsoName
   {
      uint32_t Identity : 21;
      uint16_t ManufacturerCode : 11;
      uint8_t ECUInstance : 3;
      uint8_t FunctionInstance : 5;
      uint8_t Function : 8;
      uint8_t Reserved : 1;
      uint8_t VehicleSystem : 7;
      uint8_t VehicleSystemInstance : 4;
      uint8_t IndustryGroup : 3;
      uint8_t ArbitrationCapable : 1;

      IsoName() { Reset(); }
      void Reset()
      {
         Identity = 0;
         ManufacturerCode = 0;
         ECUInstance = FunctionInstance = Function = Reserved = VehicleSystem = VehicleSystemInstance = IndustryGroup = ArbitrationCapable = 0;
      }

   };

   struct AddrClaimConfig
   {
      std::vector<addr_t> PreferredAddresses;
      IsoName Name;
      bus_t Bus;
      std::string TLA;
   };

   int operator==(const IsoName& rhs, const IsoName& lhs);

   enum { GLOBAL_ADDRESS = 0xFF};
   enum { NULL_ADDRESS = 0xFE};
   enum { INVALID_NODE = 0xFFFF};
   enum { REQUEST_PGN = 0xEA00};
   enum { ADDRESS_CLAIM_PGN = 0xEE00};
   enum { DEFAULT_PRIORITY = 6};

   struct CANMsg
   {
      addr_t Source;
      addr_t Destination;
      bus_t Bus;
      pgn_t Pgn;
      uint32_t Length;
      int64_t Timestamp;
      uint8_t Priority;
      node_t SourceDeviceID;
      node_t DestinationDeviceID;
      uint8_t* Data;
   };

   template<typename T>
   const sc::CANMsg ToCANMsg(const T& msg)
   {
      sc::CANMsg out;
      out.Pgn = T::PGN;
      out.Data = const_cast<uint8_t*>(static_cast<const uint8_t*>(msg.Data.DataBuffer()));
      out.Length = msg.Data.GetLength();
      return out;
   }

   template<typename T>
   sc::CANMsg CANMsgCast(const T& msg)
   {
      sc::CANMsg out;
      out.Pgn = T::PGN;
      out.Data = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(&msg));
      out.Length = sizeof(T);
      return out;
   }

   template<typename T>
   constexpr const T& ToMsg(const CANMsg& msg)
   {
      return *(reinterpret_cast<const T*>(static_cast<const void*>(msg.Data)));
   }

   struct CANDevice
   {
      addr_t Address = 0xFF;
      bus_t Bus = 0xFF;
      node_t Node = 0xFF;
      IsoName Name {};
   };

   using std::placeholders::_1;

   typedef std::function<void (void)> ClaimCallback;

   typedef std::vector<CANDevice> DeviceVec;

   typedef std::function<void (const CANMsg&)> MsgCallbackFn;

   typedef std::function<void (bool, sc::bus_t, sc::pgn_t, bool)> TPStatusCallbackFn;

   typedef void* MsgSubscription;
   typedef void* ClaimSubscription;
   typedef void* TPStatusSubscription;

   enum EErrorCode
   {
      EC_NoError = 0,
      EC_Failed = 1,
      EC_NotClaimed = 1,
      EC_UnknownError = 2
   };

   enum EClaimStatus
   {
      //
      // These are valid for both the address claim callback and the return for
      // GetAddressClaimStatus(). This group may be combined using a bit-wise OR.
      //
      CS_IN_PROCESS       = (1 << 0),   //!< Address Claim is still in process.
      CS_SUCCESS          = (1 << 1),   //!< Address Claim was successful.
      CS_FAIL             = (1 << 2),   //!< Address Claim has failed.
      CS_IDENTICAL_NAME   = (1 << 3),   //!< Another device with an identical NAME
      //!  was detected. This is always paired with
      //!  ISO_AC_FAIL to provide more info about why
      //!  the claim failed.

      //
      // These are valid only for the return value of GetAddressClaimStatus().
      //
      CS_UNCLAIMED        = (1 << 4),   //!< The node handle is valid but the node has
      //!  not reclaimed its address since the system
      //!  went to sleep mode and woke up.
   };

   #define ASSERT_SIZE(_MSG, _SIZE_TYPE) static_assert(sizeof(_MSG) == _SIZE_TYPE, "Message size wrong!!! Use Packing.")
   #define ASSERT_EIGHT(_MSG) static_assert(sizeof(_MSG) == 8, "Message is Not 8-Bytes!!! Use Packing.")
}
