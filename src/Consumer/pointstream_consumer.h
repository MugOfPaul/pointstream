#pragma once

#include "pointstream_common.h"
#include <asio.hpp>
#include <memory>
#include <queue>


//////////////////////////////////////////////////////////////////////////////
class PointStreamConsumer  {
public:
  PointStreamConsumer();
  virtual ~PointStreamConsumer();
  void Start(std::string& host, short port);
  void Update();
  void Stop();

  PointStreamPointBuffer& GetPoints();

private:
  void ProcessFullPacket(PointStreamCommandPacket& packet);
  void ReadPacketPayload();
  void ReadNewPacket();
  void ReadDataStream();
  void Write(PointStreamCommandPacket& packet);
  void SendVersion();

 
private:
  asio::io_service          io_service;

  // TCP stuff
  asio::ip::tcp::socket     socket; 
  PointStreamCommandPacket         scratch_packet;
  std::queue<PointStreamCommandPacket> packets;

  //UDP stuff
  asio::ip::udp::socket               data_socket;
  PointStreamPointBuffer*             data_buffer;
  PointStreamPointBuffer              scratch_buffers[2];
  unsigned char                       scratch_buffer_index;
  std::mutex                          data_buffer_mutex;
  int                                 data_points_to_read;
  unsigned char                       chunk_buffer[kMaxPointsInMTU * sizeof(PointStreamPoint)];

};