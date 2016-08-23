#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "pointstream_processor.h"

PointStreamProcessor::PointStreamProcessor():
   raw_cloud(nullptr) 
  ,filter_cloud(nullptr)
{
}


PointStreamProcessor::~PointStreamProcessor() {
  filter_cloud = nullptr;
  raw_cloud = nullptr;
}

ColorPointCloudPtr PointStreamProcessor::GetLatestCloudFiltered() {
  std::lock_guard<std::mutex> lock(filter_cloud_mutex);
  return filter_cloud;
}

// PassThrough to set min/max depth
// http://pointclouds.org/documentation/tutorials/passthrough.php
static pcl::PassThrough<ColorPoint> filterPassThrough_;

 // Voxel Grid to constrain data to known structure
static pcl::ApproximateVoxelGrid<ColorPoint> filterVoxel_;


void PointStreamProcessor::Initialize(short w, short h) {
  filter_cloud = ColorPointCloudPtr(new ColorPointCloud);
  raw_cloud = ColorPointCloudPtr(new ColorPointCloud(w, h));
  raw_cloud->is_dense = false;

  filterPassThrough_.setInputCloud(raw_cloud); // first filter so it takes the raw cloud
  filterPassThrough_.setFilterFieldName("z"); 
  filterPassThrough_.setFilterLimits(0.001, 10);
  filterPassThrough_.setKeepOrganized(true); // this keeps the outliers and sets them to NaN

  filterVoxel_.setInputCloud(filter_cloud);
  filterVoxel_.setLeafSize(0.01f, 0.01f, 0.01f); // Value is meters
}

void PointStreamProcessor::BeginFrame() {
  filter_cloud_mutex.lock();
}

void PointStreamProcessor::SubmitPoint(int index, float x, float y, float z, float rgb) {
  raw_cloud->points[index].x = x;
  raw_cloud->points[index].y = y;
  raw_cloud->points[index].z = z;
  raw_cloud->points[index].rgb = rgb;
}

void PointStreamProcessor::EndFrame() {
  //std::lock_guard<std::mutex> lock(filter_cloud_mutex);
  RunFilters();
  filter_cloud_mutex.unlock();
}

void PointStreamProcessor::RunFilters() {

  filterPassThrough_.filter(*filter_cloud);   
  filterVoxel_.filter (*filter_cloud); 
}

