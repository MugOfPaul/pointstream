#pragma once

#include <pcl/common/common_headers.h>
#include <asio.hpp>

static double kCurrentVersion = 0.1;

//////////////////////////////////////////////////////////////////////////////
// Helper function to pack a 32-bit color
inline float pack_color(uint8_t r, uint8_t g, uint8_t b) {
  int32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  return *reinterpret_cast<float*>(&rgb);
}

//////////////////////////////////////////////////////////////////////////////
typedef pcl::Normal Normal;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef NormalCloud::Ptr NormalCloudPtr;

typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;
typedef ColorPointCloud::Ptr ColorPointCloudPtr;


//////////////////////////////////////////////////////////////////////////////
struct PointStreamPoint {
  unsigned int index;
  float r, g, b, color;
};

//////////////////////////////////////////////////////////////////////////////
class PointStreamPacket {
public:
  enum PacketType : std::int8_t {
      Unknown = -1,
      Version,
      ParameSet,
      ParamGet,
      PointCloudStart
  };
  enum { kHeaderLengthBytes   = sizeof(std::int8_t) + sizeof(std::size_t) };
  enum { kMaxPayloadBytes = 512 - kHeaderLengthBytes };

  // Convenience constructor for simple types
  PointStreamPacket(PacketType t) : type(t) {
    
    // Pack for known types
    if (type == Version) {
      Pack(type, &kCurrentVersion, sizeof(kCurrentVersion));
    }
  }

  // Default constructor
  PointStreamPacket() : type(Unknown), payload_size(0) {
  }

  unsigned char* Payload() {
    return payload + kHeaderLengthBytes;
  }

  std::size_t PayloadSize() const {
    return payload_size;
  }

  std::size_t Size() const {
    return kHeaderLengthBytes + PayloadSize();
  }

  unsigned char* Raw() {
    return payload;
  }

  PacketType Type() const {
    return type;
  }

  void Unpack() {
    type = *((PacketType*)payload);
    payload_size = *((std::size_t*)(payload + sizeof(std::int8_t)));
    payload_size = std::min<std::size_t>(payload_size, kMaxPayloadBytes);
  }

  void Reset() {
    type = Unknown;
    payload_size = 0;
    std::memset(Payload(), 0, kMaxPayloadBytes);
  }

  void Pack(PacketType t, void* data, std::size_t data_len) {
    type = t;
    payload_size = std::min<std::size_t>(data_len, kMaxPayloadBytes);

    // get the actual data in the payload part of the buffer
    std::memcpy(Payload(), data, payload_size);
    // put the header together
    *((PacketType*)payload) = t;
    *((std::size_t*)(payload + sizeof(std::int8_t)))  = payload_size;

  }


private:
  PacketType type;
  std::size_t payload_size;
  unsigned char payload[ kHeaderLengthBytes +  kMaxPayloadBytes ];

};

typedef std::vector<PointStreamPoint> PointStreamPointBuffer;



