#pragma once
#include "ICanSource.hpp"
#include <fstream>
#include <iostream>
#include <memory>
#include <boost/interprocess/ipc/message_queue.hpp>

class CanPlayer : public ICanSource
{
private:
  sc::MessageInstance_t Previous;
  sc::MessageInstance_t Current;
  std::ifstream File;
  bool More = true;

  void Close();
  bool ProcessMsg();
  bool ProcessLog(bool& isValidOut, sc::MessageInstance_t& messageOut);
  static bool IsValid(const sc::CanMessage_t& msg);
public:
  CanPlayer() {}
  virtual ~CanPlayer();

  bool Open(const char * path);
  bool Step(bool& isValidOut, sc::MessageInstance_t& messageOut) override;
};