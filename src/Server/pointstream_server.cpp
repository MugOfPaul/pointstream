
#include "pointstream_server.h"
#include "pointstream_common.h"

#include <iostream>
#include <memory>

using asio::ip::tcp;


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
PointStreamConnection::PointStreamConnection(int i, PointStreamServer* server, asio::ip::tcp::socket s)
:id(i)
,server(server)
,socket(std::move(s))
{
}
  
//////////////////////////////////////////////////////////////////////////////
PointStreamConnection::~PointStreamConnection() {
  //socket.close();
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::Start() {
  ReadNewPacket();
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::ProcessFullPacket() {

  // Client has sent us their version information
  if (scratch_packet.Type() == PointStreamPacket::Version) {
    double clientVersion = *((double*)scratch_packet.Payload());
    std::cout << "Client " << id << " is version: " << clientVersion << std::endl;
    // let them know our version
    SendVersion();
  }
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::SendVersion() {
  PointStreamPacket response(PointStreamPacket::Version);
  Write(response);
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::ReadPacketPayload() {
  auto self(shared_from_this());
  asio::async_read(socket,
      asio::buffer(scratch_packet.Payload(), scratch_packet.PayloadSize()),
      [this, self](std::error_code ec, std::size_t /*length*/)
      {
        if (!ec)
        {
          ProcessFullPacket();
          ReadNewPacket();
        }
        else
        {
          std::cout << "Error Reading Payload from Client: " << ec.message() << " (" << ec.value() << ")" << std::endl;
        }
      });
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::ReadNewPacket() {

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
          std::cout << "Error Reading Packet from Client: " << ec.message() << " (" << ec.value() << ")" << std::endl;
          server->RemoveConnection(this);
        }
      });
}


//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::Write(PointStreamPacket& packet) {
  asio::async_write(socket,
      asio::buffer(packet.Raw(), packet.Size()),
      [this, packet](std::error_code ec, std::size_t /*length*/)
      {
        if (!ec)
        {
           //std::cout << "Sent Packet to Client: " << packet.Type() << " (" << packet.PayloadSize() << " payload bytes)" << std::endl;
        }
        else
        {
          std::cout << "Error Sending Packet to Client: " << ec.message() << " (" << ec.value() << ")" << std::endl;
        }
      });
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
PointStreamServer::PointStreamServer(short port_)
  :port(port_)
  ,io_service()
  ,acceptor(io_service, tcp::endpoint(tcp::v4(), port_))
  ,socket(io_service)
{
  
}

//////////////////////////////////////////////////////////////////////////////
PointStreamServer::~PointStreamServer() {
  //connections.clear();
  Stop();
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamServer::Update() {
  io_service.poll();
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamServer::Start() {
  RunAccept();
}


//////////////////////////////////////////////////////////////////////////////
void PointStreamServer::Stop() {
   io_service.stop();
   socket.close();
   connections.clear();
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamServer::RemoveConnection(PointStreamConnection* conn) {
  connections.erase(std::find_if(connections.begin(), connections.end(),
    [&] (std::shared_ptr<PointStreamConnection>& p) {
      return p.get() == conn;
    }
    ));
  std::cout << connections.size() << " active connections." << std::endl;
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamServer::RunAccept() {
  std::cout << connections.size() << " active connections." << std::endl;
  acceptor.async_accept(socket, 
    [this](std::error_code ec) {
      if (!ec)
      {
        std::cout << "New connection. From " << socket.remote_endpoint() << " to " << socket.local_endpoint() << std::endl;

        auto conn = std::make_shared<PointStreamConnection>(connections.size(), this, std::move(socket));
        connections.push_back(conn);
        conn->Start();
      } else {
        std::cout << "Connection Error. " << ec << std::endl;
      }

      RunAccept();
    });
}