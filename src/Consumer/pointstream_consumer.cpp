
#include "pointstream_consumer.h"

#include <iostream>
#include <memory>

using asio::ip::tcp;


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
PointStreamConsumer::PointStreamConsumer()
  :io_service()
  ,socket(io_service)
{
  
}

//////////////////////////////////////////////////////////////////////////////
PointStreamConsumer::~PointStreamConsumer() {
  //connections.clear();
  Stop();
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::Update() {
  io_service.run_one();
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::Start(std::string& host, short port) {

  tcp::resolver resolver(io_service);
  std::cout << "Waiting to connect to " << host << ":" << port << std::endl;
  asio::connect(socket, resolver.resolve(tcp::resolver::query(host, std::to_string(port))));
  std::cout << "Connected to " << socket.remote_endpoint() << std::endl;
}


//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::Stop() {
   socket.close();
   io_service.stop();
}
