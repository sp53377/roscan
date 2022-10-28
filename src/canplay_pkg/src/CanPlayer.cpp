///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#include "CanPlayer.hpp"
#include "ProcessingFns.hpp"
#include <chrono>
#include <thread>

using namespace std;
using namespace std::chrono;

CanPlayer::~CanPlayer()
{
  Close();
}

bool CanPlayer::Open(const char * path)
{
  File.open(path);
  return File.is_open();
}

void CanPlayer::Close()
{
  File.close();
}

bool CanPlayer::IsValid(const sc::CanMessage_t& msg)
{
  return msg.Channel >= 0 && msg.Length > 0 && msg.Length <= 8;
}

bool CanPlayer::ProcessMsg()
{
  bool isValid = false;
  if (IsValid(Previous.Msg)) {
    double delta = Current.Timestamp - Previous.Timestamp;
    if (delta > 0.001) {
      int sleepTimeMs = static_cast<int>(delta * 1000);
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepTimeMs));
    }
    isValid = true;
  }
  return isValid;
}

bool CanPlayer::ProcessLog(bool& isValidOut, MessageInstance_t& messageOut)
{
  bool more = false;
  std::string str;
  if (std::getline(File, str)) {
    sc::CanMessage_t msg;
    Previous.Timestamp = Current.Timestamp;
    Current.Timestamp = processing::ProcessLine(str, msg);
    isValidOut = ProcessMsg();
    if(isValidOut)
    {
	messageOut = Previous;
    }
    Previous.Msg = msg;
    more = true;
  }
  return more;
}

bool CanPlayer::TryReceive(bool& isValidOut, MessageInstance_t& messageOut)
{
  bool more = More;
  More = ProcessLog(isValidOut, messageOut);
  return more;
}

bool CanPlayer::Send(sc::CanMessage_t& msg)
{
  (void)msg;
  return true;
}