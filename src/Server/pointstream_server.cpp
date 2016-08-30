
#include "pointstream_server.h"
#include "pointstream_common.h"

#include <iostream>
#include <memory>
#include <iterator>
#include <algorithm>
#include <chrono>

using asio::ip::tcp;


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
PointStreamConnection::PointStreamConnection(int i, PointStreamServer* server, asio::ip::tcp::socket cmd_sock, asio::ip::udp::socket data_sock)
:id(i)
,server(server)
,packet_socket(std::move(cmd_sock))
,data_socket(std::move(data_sock))
,data_endpoint()
,data_ready(false)
,points_out(nullptr)
{
}
  
//////////////////////////////////////////////////////////////////////////////
PointStreamConnection::~PointStreamConnection() {
  //socket.close();
  data_socket.close();
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::Start() {

  // Over the TCP socket, send our version and wait for response
  SendVersion();
  ReadNewPacket();
}

void PointStreamConnection::Stop() {
  if (data_socket.is_open()) {
    data_socket.close();
  }
  server->RemoveConnection(this);
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::Update() {
  while (!packets.empty()) {
    ProcessFullPacket(packets.front());
    packets.pop();
  }
}


void PointStreamConnection::StartDataSocket() {

  // Send the connection info over the TCP socket, then set up the accept loop
  PointStreamCommandPacket info;

  DataConnectionInfo conn_info;
  asio::ip::udp::endpoint endpoint = data_socket.local_endpoint();
  conn_info.port = endpoint.port();

  std::string addr(endpoint.address().to_string());
  std::memcpy(conn_info.address, addr.c_str(), std::min(sizeof(conn_info.address) - 1, addr.length() + 1));

  info.Pack(PointStreamCommandPacket::DataConnection, &conn_info, sizeof(conn_info));
  WritePacket(info);

  std::cout << "Data connection for " << id << " is " << addr << ":" << conn_info.port << std::endl;
 
  // we do a junk receive to set up the endpoint
  // TODO: either use this as an authentication step or find a cleaner way
  char junk_buffer[64];
  data_socket.async_receive_from(
        asio::buffer(junk_buffer, 64), data_endpoint,
        [this, junk_buffer](std::error_code ec, std::size_t bytes_recvd)
        {
          if (!ec && bytes_recvd > 0)
          {
            data_ready = true;
            //WriteStreamData();
          } else {
            std::cout << "Error Reading Data from Client: " << ec.message() << " (" << ec.value() << ")" << std::endl;
          }
        });
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::ProcessFullPacket(PointStreamCommandPacket& packet) {

  // Client has sent us their version information
  if (packet.Type() == PointStreamCommandPacket::Version) {
    double clientVersion = *((double*)packet.Payload());
    std::cout << "Client " << id << " is version: " << clientVersion << std::endl;
    
    // Set up the UDP socket
    StartDataSocket(); 
  }
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::SendVersion() {
  PointStreamCommandPacket response(PointStreamCommandPacket::Version);
  WritePacket(response);
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::SendPointBuffer(PointStreamPointBuffer* buffer) {

  if (data_ready) {

    data_ready = false;
    points_out = buffer;
    // Give a heads up on the incoming data
    PointCloudInfo cloud_info;
    cloud_info.number_points = points_out->size();
    PointStreamCommandPacket cloud_info_pkt;
    cloud_info_pkt.Pack(PointStreamCommandPacket::PointCloudInfo, &cloud_info, sizeof(cloud_info));
    WritePacket(cloud_info_pkt);

    static auto last_time = std::chrono::steady_clock::now();
    auto diff = std::chrono::steady_clock::now() - last_time;
    last_time = std::chrono::steady_clock::now();
    //std::cout << "Transmitting " << (1000.0 / std::chrono::duration<double, std::milli>(diff).count()) << "fps (Point Count: " << cloud_info.number_points << ")" << std::endl;


    //std::cout << "New Point Cloud to Send. Point Count: " << cloud_info.number_points << std::endl;

    // Kick off the data stream
    StartNewStreamWrite();
  }

}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::ReadPacketPayload() {
  auto self(shared_from_this());
  asio::async_read(packet_socket,
      asio::buffer(scratch_packet.Payload(), scratch_packet.PayloadSize()),
      [this, self](std::error_code ec, std::size_t /*length*/)
      {
        if (!ec)
        {
          packets.push(scratch_packet);
          ReadNewPacket();
        }
        else
        {
          std::cout << "Error Reading Payload from Client: " << ec.message() << " (" << ec.value() << ")" << std::endl;
          Stop();
        }
      });
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::ReadNewPacket() {

  scratch_packet.Reset();
  asio::async_read(packet_socket,
      asio::buffer(scratch_packet.Raw(), PointStreamCommandPacket::kHeaderLengthBytes),
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
          Stop();
        }
      });
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::StartNewStreamWrite() {
  data_cursor = points_out->begin();
  WriteStreamData();
}


//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::WriteStreamData() {

  int points_cnt = std::min((unsigned int)std::distance(data_cursor, points_out->end()), kMaxPointsInMTU);

  data_socket.async_send_to(
      asio::buffer(&*data_cursor, points_cnt * sizeof(PointStreamPoint)), data_endpoint,
      [this, points_cnt](std::error_code ec, std::size_t len)
      {
        if (!ec)
        {
          //std::cout << "Sent Data to Client: " << len << " bytes" << std::endl;

          // check to see if we have more to send
          data_cursor += points_cnt;
          if (data_cursor != points_out->end()) {
            WriteStreamData();
          } else {
            // signal we are ready for new data
            data_ready = true;
          }
        }
        else
        {
          std::cout << "Error Sending Stream Data to Client: " << ec.message() << " (" << ec.value() << ")" << std::endl;
          StartDataSocket();
        }
      });
}


//////////////////////////////////////////////////////////////////////////////
void PointStreamConnection::WritePacket(PointStreamCommandPacket& packet) {
  asio::async_write(packet_socket,
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

  for (auto conn : connections) {
    conn->Update();
  }
}


//////////////////////////////////////////////////////////////////////////////
void PointStreamServer::SetOutgoingPointBuffer(PointStreamPointBuffer& buffer) {

  // cache
  points_out = buffer;

  for (auto conn : connections) {
    conn->SendPointBuffer(&points_out);
  }

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
        
        // Set up a new connection with data socket
        int id = connections.size();
        unsigned short data_port = port + id;
        asio::ip::udp::socket udp_socket(io_service, asio::ip::udp::endpoint(asio::ip::udp::v4(), data_port));
        auto conn = std::make_shared<PointStreamConnection>(id, this, std::move(socket), std::move(udp_socket));
        
        connections.push_back(conn);
        conn->Start();
      } else {
        std::cout << "Connection Error. " << ec << std::endl;
      }

      RunAccept();
    });
}