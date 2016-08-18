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
  void StartListening();
  void Stop();
  void Update();
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
  PointStreamConnection(PointStreamServer* server, asio::ip::tcp::socket s);
  virtual ~PointStreamConnection();
  void Start();
  asio::ip::tcp::socket& Socket() { return socket; }

  void Read();
  void AsyncWrite(PointStreamPointBuffer& buffer);

private:
  PointStreamServer* server;
  asio::ip::tcp::socket   socket; 
};
