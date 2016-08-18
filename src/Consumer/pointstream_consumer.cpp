
#include "pointstream_consumer.h"
#include "pointstream_common.h"

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

void PointStreamConsumer::Read() {
  PointStreamPointBuffer p;
  socket.async_receive(asio::buffer(p),
      [this, p](std::error_code ec, std::size_t length)
        {
          if (!ec)
          {
            std::cout << "Read " << length << " bytes. " << std::endl;
            Read();
          } else {
            if (ec.value() == 2) {
              std::cout << "Remote Disconnected." << std::endl;
              Stop();
            } else {
              std::cout << "Error: " << ec.value() << std::endl;
            }
          }

        });
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
  Read();
}


//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::Stop() {
   socket.close();
   io_service.stop();
}
