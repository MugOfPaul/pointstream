#pragma once

#include <pcl/common/common_headers.h>

typedef pcl::Normal Normal;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef NormalCloud::Ptr NormalCloudPtr;

typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;
typedef ColorPointCloud::Ptr ColorPointCloudPtr;

enum PointStreamMsgType { 
  kMSG_POINTCLOUDINFO,
  kMSG_POINTCLOUDPOINT
};


//////////////////////////////////////////////////////////////////////////////
// Helper function to pack a 32-bit color
inline float pack_color(uint8_t r, uint8_t g, uint8_t b) {
  int32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  return *reinterpret_cast<float*>(&rgb);
}

