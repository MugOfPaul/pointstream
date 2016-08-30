
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
  ,data_socket(io_service)
  ,scratch_buffer_index(0)
  ,data_buffer(nullptr)
{
  data_buffer = &scratch_buffers[scratch_buffer_index];
}

//////////////////////////////////////////////////////////////////////////////
PointStreamConsumer::~PointStreamConsumer() {
  //connections.clear();
  Stop();
}


//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::Update() {
  io_service.poll();

  while (!packets.empty()) {
    ProcessFullPacket(packets.front());
    packets.pop();
  }

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
            ReadNewPacket();
          } 
          else
          {
            std::cout << "Error Connecting to Server: " << ec.message() << " (" << ec.value() << ")" << std::endl;
          }
        });
  
}

//////////////////////////////////////////////////////////////////////////////
PointStreamPointBuffer& PointStreamConsumer::GetPoints() { 
  return scratch_buffers[1 - scratch_buffer_index]; 
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::ProcessFullPacket(PointStreamCommandPacket& packet) {

  // Server has sent it's version (usually a response to our version packet)
  if (packet.Type() == PointStreamCommandPacket::Version) {
    
    double serverVersion = *((double*)packet.Payload());
    std::cout << "Server is version: " << serverVersion << std::endl;

    // Reply with our version
    SendVersion();

  } else if (packet.Type() == PointStreamCommandPacket::PointCloudInfo) {
    PointCloudInfo* info = reinterpret_cast<PointCloudInfo*>(packet.Payload());
    //std::cout << "Point Cloud information received. Point Count:" << info->number_points << std::endl;

    static auto last_time = std::chrono::steady_clock::now();
    auto diff = std::chrono::steady_clock::now() - last_time;
    last_time = std::chrono::steady_clock::now();
    //std::cout << "Receiving " << (1000.0 / std::chrono::duration<double, std::milli>(diff).count()) << "fps (Point Count: " << info->number_points << ")" << std::endl;
    
    // swap the front/back buffer
    data_buffer_mutex.lock();
    scratch_buffer_index = 1 - scratch_buffer_index;
    data_buffer = &scratch_buffers[scratch_buffer_index];

    data_buffer->clear();
    data_buffer->reserve(info->number_points);
    data_points_to_read = info->number_points;
    data_buffer_mutex.unlock();

    ReadDataStream();
  } else if (packet.Type() == PointStreamCommandPacket::DataConnection) {
    
    DataConnectionInfo* info = reinterpret_cast<DataConnectionInfo*>(packet.Payload());
    std::cout << "Data connection information received" << std::endl;
    std::cout << "\t Port: " << info->port << " Address:" << info->address << std::endl;

    // Set up the UDP connection
    data_socket.close();
    data_socket.open(asio::ip::udp::v4());

    asio::ip::udp::endpoint endpoint(asio::ip::address::from_string(info->address), info->port);

    data_socket.async_connect(endpoint,
      [this](std::error_code ec)
      {
        if (!ec) {
          std::string data("Hello Data Stream");
          data_socket.send(asio::buffer(data.c_str(), data.length() + 1));
          //ReadDataStream();
        } else { 
          std::cout << "Error Connecting To Data Stream: " << ec.message() << " (" << ec.value() << ")" << std::endl;
        }
      });

  }
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::ReadDataStream() {

  data_socket.async_receive(asio::buffer(&chunk_buffer, sizeof(chunk_buffer)),
      [this](std::error_code ec, std::size_t len)
      {
        if (!ec)
        {
          unsigned int num_points = len / sizeof(PointStreamPoint);
          unsigned char* chunk_cursor = &chunk_buffer[0];
          data_points_to_read -= num_points;
          //std::cout << "Read New Points: " << num_points << " (" << len << " bytes)" << std::endl;

          data_buffer_mutex.lock();

          for (unsigned int p = 0; p < num_points; p++) {
            data_buffer->push_back(*(PointStreamPoint*)chunk_cursor);
            chunk_cursor += sizeof(PointStreamPoint);
          }
        

          data_buffer_mutex.unlock();
                    
          if (data_points_to_read > 0)
            ReadDataStream();
        }
        else
        {
          std::cout << "Error Reading Payload from Server: " << ec.message() << " (" << ec.value() << ")" << std::endl;
        }
      });
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::ReadPacketPayload() {
  asio::async_read(socket,
      asio::buffer(scratch_packet.Payload(), scratch_packet.PayloadSize()),
      [this](std::error_code ec, std::size_t /*length*/)
      {
        if (!ec)
        {
          packets.push(scratch_packet);
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
          std::cout << "Error Reading Packet from Server: " << ec.message() << " (" << ec.value() << ")" << std::endl;
        }
      });
}


//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::SendVersion() {
  PointStreamCommandPacket response(PointStreamCommandPacket::Version);
  Write(response);
}

//////////////////////////////////////////////////////////////////////////////
void PointStreamConsumer::Write(PointStreamCommandPacket& packet) {
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
