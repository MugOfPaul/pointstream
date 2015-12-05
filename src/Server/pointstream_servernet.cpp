
#include "pointstream_servernet.h"

#include <iostream>
#include <memory>

using asio::ip::tcp;


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
PointStreamConnection::PointStreamConnection(PointStreamServer* server, asio::ip::tcp::socket s)
:server(server)
,socket(std::move(s))
{
}
  
//////////////////////////////////////////////////////////////////////////////
PointStreamConnection::~PointStreamConnection() {
  //socket.close();
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::Start() {
  Read();
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::Read() {
  socket.async_read_some(asio::buffer(data_, max_length),
      [this](std::error_code ec, std::size_t length)
        {
          if (!ec)
          {
            std::cout << "Read " << length << " bytes" << std::endl;
            Write(length);
          } else {
            if (ec.value() == 2) {
              std::cout << "Client Disconnected." << std::endl;
              server->RemoveConnection(this);
            } else {
              std::cout << "Error: " << ec.value() << std::endl;
            }
          }

        });
}


//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::Write(std::size_t length) {
  asio::async_write(socket, asio::buffer(data_, length),
      [this](std::error_code ec, std::size_t length)
      {
        if (!ec)
        {
          std::cout << "Wrote " << length << " bytes" << std::endl;
          Read();
        } else {
          if (ec.value() == 2) {
            std::cout << "Client Disconnected." << std::endl;
            server->RemoveConnection(this);
          } else {
            std::cout << "Error: " << ec.value() << std::endl;
          }
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
  acceptor.get_io_service().run_one();
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamServer::StartListening() {
  RunAccept();
}


//////////////////////////////////////////////////////////////////////////////
void PointStreamServer::Stop() {
   socket.close();
   acceptor.get_io_service().stop();
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
        std::cout << "New connection. " << socket.remote_endpoint() << std::endl;

        auto conn = std::make_shared<PointStreamConnection>(this, std::move(socket));
        connections.push_back(conn);
        conn->Start();
      } else {
        std::cout << "Connection Error. " << ec << std::endl;
      }

      RunAccept();
    });
}