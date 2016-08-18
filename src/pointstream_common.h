#pragma once

#include <pcl/common/common_headers.h>
#include <asio.hpp>

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


/**
//////////////////////////////////////////////////////////////////////////////
template <typename T>
class AsioBuffer {
public:
 
  explicit AsioBuffer()
    : data_(new std::vector<T>())
    , buffer_(asio::buffer(*data_))
  {
  }

  void push_back(const T& p) {
    data_->push_back(p);
  }

  void clear() {
    data_->clear();
  }

  // Implement the ConstBufferSequence requirements.
  typedef asio::const_buffer value_type;
  typedef const asio::const_buffer* const_iterator;
  const asio::const_buffer* begin() const { return &buffer_; }
  const asio::const_buffer* end() const { return &buffer_ + 1; }

private:
  std::shared_ptr<std::vector<T>> data_;
  asio::const_buffer buffer_;
};

//typedef AsioBuffer<PointStreamPoint> PointStreamPointBuffer;
**/

typedef std::vector<PointStreamPoint> PointStreamPointBuffer;




//////////////////////////////////////////////////////////////////////////////
// Helper function to pack a 32-bit color
inline float pack_color(uint8_t r, uint8_t g, uint8_t b) {
  int32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  return *reinterpret_cast<float*>(&rgb);
}

