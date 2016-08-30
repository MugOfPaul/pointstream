#pragma once


#include "pointstream_common.h"
#include <asio.hpp>
#include <memory>
#include <queue>

class PointStreamConnection;


//////////////////////////////////////////////////////////////////////////////
class PointStreamServer  {
public:
  PointStreamServer(short port);
  virtual ~PointStreamServer();
  void Start();
  void Update();
  void Stop();
  void RemoveConnection(PointStreamConnection* conn);

  void SetOutgoingPointBuffer(PointStreamPointBuffer& points);
  PointStreamPointBuffer& GetOutgoingPointBuffer() { return points_out; }

private:
  void RunAccept();

  short port;
  asio::io_service          io_service;
  asio::ip::tcp::acceptor   acceptor;
  asio::ip::tcp::socket     socket; 

  std::vector<std::shared_ptr<PointStreamConnection>> connections;

  PointStreamPointBuffer points_out;
  
};


//////////////////////////////////////////////////////////////////////////////
class PointStreamConnection : public std::enable_shared_from_this<PointStreamConnection> {
public:
  PointStreamConnection(int id, PointStreamServer* server, asio::ip::tcp::socket cmd_sock, asio::ip::udp::socket data_sock);
  virtual ~PointStreamConnection();
  void Start();
  void Update();
  void Stop();
  void SendPointBuffer(PointStreamPointBuffer* buffer);
  

private:
  void ReadNewPacket();
  void ReadPacketPayload();
  void ProcessFullPacket(PointStreamCommandPacket& packet);
  void WritePacket(PointStreamCommandPacket& packet);

  void SendVersion();
  void StartDataSocket();
  void StartNewStreamWrite();
  void WriteStreamData();

private:
  int id;
  PointStreamServer*                server;
  PointStreamPointBuffer*           points_out;
  
  // TCP stuff
  asio::ip::tcp::socket          packet_socket; 
  PointStreamCommandPacket       scratch_packet;
  std::queue<PointStreamCommandPacket> packets;

  // UDP stuff
  asio::ip::udp::socket             data_socket;
  asio::ip::udp::endpoint           data_endpoint;
  bool                              data_ready;
  PointStreamPointBuffer::iterator  data_cursor;
};
