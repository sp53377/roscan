#include "ProcessingFns.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace std;

template <typename T>
bool FromHex(const string &hexValue, T &result)
{
  std::stringstream ss;
  ss << std::hex << hexValue;
  ss >> result;
  return !ss.fail();
}

template <> bool FromHex<int8_t>(const string &hexValue, int8_t&result)
{
  std::stringstream ss;
  ss << std::hex << hexValue;
  int temp=-1;
  ss >> temp;
  result = temp;
  return !ss.fail();
}

template <> bool FromHex<uint8_t>(const string &hexValue, uint8_t&result)
{
  std::stringstream ss;
  ss << std::hex << hexValue;
  int temp=-1;
  ss >> temp;
  result = temp;
  return !ss.fail();
}

bool GetTimestamp(const std::string &str, double &timestamp)
{
  timestamp = atof(str.c_str());
  return (timestamp > 0.0);
}

double processing::ProcessLine(const std::string &line, sc::CanMessage_t& msg)
{
  istringstream ss(line);
  string str;
  double timestamp = 0.0;
  int length = -1;
  msg.Channel = -1;
  msg.FrameId = 0;
  msg.Length = 0;
  while (getline(ss, str, ' '))
  {
    if (msg.Channel < 0)
    {
      if(str.length() == 2)
      {
        FromHex(str, msg.Channel);
      }
    }
    else if (!msg.FrameId)
    {
      if (str.length() == 8)
      {
        if (!FromHex(str, msg.FrameId))
        {
          break;
        }
      }
    }
    else if (length < 0)
    {
      if (FromHex(str, length))
      {
        msg.Length = length;
      }
      else
      {
        break;
      }
    }
    else if (length > 0 && length <= 8)
    {
      uint8_t byte;
      if (FromHex(str, byte))
      {
        msg.Bytes[8-length] = byte;
      }
      else
      {
        break;
      }
      length--;
    }
    else if (length == 0 && timestamp == 0.0)
    {
      GetTimestamp(str, timestamp);
    }
  }
  return timestamp;
}
