#pragma once


#include "pointstream_common.h"
#include <asio.hpp>
#include <memory>

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
  PointStreamConnection(int id, PointStreamServer* server, asio::ip::tcp::socket s);
  virtual ~PointStreamConnection();
  void Start();
  asio::ip::tcp::socket& Socket() { return socket; }

private:
  void ReadNewPacket();
  void ReadPacketPayload();
  void ProcessFullPacket();
  void Write(PointStreamPacket& packet);
  void SendVersion();

private:
  int id;
  PointStreamServer*      server;
  asio::ip::tcp::socket   socket; 
  PointStreamPacket       scratch_packet;
};
