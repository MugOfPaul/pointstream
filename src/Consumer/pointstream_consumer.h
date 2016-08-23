#pragma once

#include "pointstream_common.h"
#include <asio.hpp>
#include <memory>


//////////////////////////////////////////////////////////////////////////////
class PointStreamConsumer  {
public:
  PointStreamConsumer();
  virtual ~PointStreamConsumer();
  void Start(std::string& host, short port);
  void Update();
  void Stop();

private:
  void ProcessFullPacket();
  void ReadPacketPayload();
  void ReadNewPacket();
  void Write(PointStreamPacket& packet);
  void SendVersion();

 
private:
  asio::io_service          io_service;
  asio::ip::tcp::socket     socket; 
  PointStreamPacket         scratch_packet;
};