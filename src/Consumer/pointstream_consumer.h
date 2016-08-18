#pragma once


#include <asio.hpp>
#include <memory>


//////////////////////////////////////////////////////////////////////////////
class PointStreamConsumer  {
public:
  PointStreamConsumer();
  virtual ~PointStreamConsumer();
  void Start(std::string& host, short port);
  void Stop();
  void Read();
  void Update();
 
private:
 
  short port;
  asio::io_service          io_service;
  asio::ip::tcp::socket     socket; 

};