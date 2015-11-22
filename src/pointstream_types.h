#pragma once

#include <pcl/common/common_headers.h>

typedef pcl::Normal Normal;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef NormalCloud::Ptr NormalCloudPtr;

typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;
typedef ColorPointCloud::Ptr ColorPointCloudPtr;
