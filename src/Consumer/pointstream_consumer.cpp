
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


//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::Update() {
  io_service.poll();
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::Start(std::string& host, short port) {

  tcp::resolver resolver(io_service);
  asio::async_connect(socket,
                      resolver.resolve(tcp::resolver::query(host, std::to_string(port))),
        [this](std::error_code ec, tcp::resolver::iterator)
        {
          if (!ec)
          {
            std::cout << "Connected to " << socket.remote_endpoint() << std::endl;
            SendVersion();
            ReadNewPacket();
          } 
          else
          {
            std::cout << "Error Connecting to Server: " << ec.message() << " (" << ec.value() << ")" << std::endl;
          }
        });
  
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::ProcessFullPacket() {

  // Server has sent it's version (usually a response to our version packet)
  if (scratch_packet.Type() == PointStreamPacket::Version) {
    double serverVersion = *((double*)scratch_packet.Payload());
    std::cout << "Server is version: " << serverVersion << std::endl;
  }
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::ReadPacketPayload() {
  asio::async_read(socket,
      asio::buffer(scratch_packet.Payload(), scratch_packet.PayloadSize()),
      [this](std::error_code ec, std::size_t /*length*/)
      {
        if (!ec)
        {
          ProcessFullPacket();
          ReadNewPacket();
        }
        else
        {
          std::cout << "Error Reading Payload from Server: " << ec.message() << " (" << ec.value() << ")" << std::endl;
        }
      });
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::ReadNewPacket() {

  scratch_packet.Reset();
  asio::async_read(socket,
      asio::buffer(scratch_packet.Raw(), PointStreamPacket::kHeaderLengthBytes),
      [this](std::error_code ec, std::size_t /*length*/)
      {
        if (!ec)
        {
          scratch_packet.Unpack();
          ReadPacketPayload();
        }
        else
        {
          std::cout << "Error Reading Packet from Server: " << ec.message() << " (" << ec.value() << ")" << std::endl;
        }
      });
}


//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::SendVersion() {
  PointStreamPacket response(PointStreamPacket::Version);
  Write(response);
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::Write(PointStreamPacket& packet) {
  asio::async_write(socket,
      asio::buffer(packet.Raw(), packet.Size()),
      [this, packet](std::error_code ec, std::size_t /*length*/)
      {
        if (!ec)
        {
           //std::cout << "Sent Packet to Server: " << packet.Type() << " (" << packet.PayloadSize() << " payload bytes)" << std::endl;
        }
        else
        {
          std::cout << "Error Sending Packet to Server: " << ec.message() << " (" << ec.value() << ")" << std::endl;
        }
      });
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::Stop() {
   io_service.stop();
   socket.close();
}
