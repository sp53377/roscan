///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#include "GpsService.h"
#include "GpsServiceROS.h"
#include <GpsService/GpsMessages.h>
#include <CANFactory/CANTypes.h>
#include <iostream>

using namespace gps;

GpsService::GpsService(std::shared_ptr<rclcpp::Node> & node)
: Node(node),
  CAN(new GpsServiceROS(node, *this)),
  Devices()
{
  EpochPublisher = Node->create_publisher<bridge::GpsEpoch::rostype>(bridge::GpsEpoch::topic, 10);
  ExPublisher = Node->create_publisher<bridge::GpsEx::rostype>(bridge::GpsEx::topic, 10);
  DevicePublisher = Node->create_publisher<bridge::GpsDevice::rostype>(bridge::GpsDevice::topic, 10);
  PgnReqPublisher = Node->create_publisher<bridge::GlobalPgnRequest::rostype>(bridge::GlobalPgnRequest::topic, 10);
}

GpsService::~GpsService()
{
  delete CAN;
  CAN = nullptr;
}

void GpsService::GpsDevicesReq(const hash_t & responseTopic)
{
  for (auto & kv:Devices) {
    GpsModel & model = kv.second;
    model.PublishDevice(responseTopic);
  }
}

GpsModel & GpsService::FindModel(sc::node_t sourceNode)
{
  GpsModel * model = &NullModel;
  ModelMap::iterator it = Devices.find(sourceNode);
  if (it == Devices.end()) {
    sc::CANDevice device = CAN->GetDevice(sourceNode);
    model = &Devices[sourceNode];
    model->SetCANDevice(device);
    model->SetGpsService(this);
  } else {
    model = &(it->second);
  }
  return *model;
}

void GpsService::HandleEpochReceived(const int64_t & timestamp)
{
  int expected = 0;
  int actual = 0;
  for (auto & pair:Devices) {
    auto & model = pair.second;
    int64_t dt = labs(model.LastEpochTimestamp() - timestamp);
    if (dt < EPOCH_TIMEOUT) {//Is this receiver active
      expected++;
      if (dt < SYNC_WINDOW) {//Is this epoch recent
        actual++;
      }
    }
  }

  if (actual >= expected) {

    //TODO static const hash_t topic = fps::ToHash(topics::gps_AllEpochsReceived::Topic);
    //TODO fps::Publish(topic, timestamp);
  }
}

void GpsService::PublishGpsEpoch(const gps::GpsEpoch & epoch)
{
  EpochPublisher->publish(bridge::GpsEpoch::ToROS(epoch));

  std::cout << "Epoch" << std::endl;
  can_interfaces::msg::PgnRequest pgnReq;
  pgnReq.pgn = 0x12345;
  pgnReq.can_data.source = 0;
  pgnReq.can_data.timestamp = 0;
  PgnReqPublisher->publish(pgnReq);
}

void GpsService::PublishGpsEx(const gps::GpsEx & ex)
{
  ExPublisher->publish(bridge::GpsEx::ToROS(ex));
}

void GpsService::PublishGpsDevice(const gps::GpsDevice & device)
{
  DevicePublisher->publish(bridge::GpsDevice::ToROS(device));
}
