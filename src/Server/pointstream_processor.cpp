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
   m_RawCloud(nullptr) 
  ,m_FilterCloud(nullptr)
{
}


PointStreamProcessor::~PointStreamProcessor() {
  m_FilterCloud = nullptr;
  m_RawCloud = nullptr;
}

ColorPointCloudPtr PointStreamProcessor::GetLatestCloudFiltered() {
  std::lock_guard<std::mutex> lock(filter_cloud_mutex);
  return m_FilterCloud;
}

void PointStreamProcessor::Initialize(short w, short h) {
  m_FilterCloud = ColorPointCloudPtr(new ColorPointCloud);
  m_RawCloud = ColorPointCloudPtr(new ColorPointCloud(w, h));
  m_RawCloud->is_dense = false;
}

void PointStreamProcessor::BeginFrame() {

}

void PointStreamProcessor::SubmitPoint(int index, float x, float y, float z, float rgb) {
  m_RawCloud->points[index].x = x;
  m_RawCloud->points[index].y = y;
  m_RawCloud->points[index].z = z;
  m_RawCloud->points[index].rgb = rgb;
}

void PointStreamProcessor::EndFrame() {

  std::lock_guard<std::mutex> lock(filter_cloud_mutex);
  RunFilters();
}

void PointStreamProcessor::RunFilters() {
  // Master filters
  pcl::PassThrough<ColorPoint> pass;
  pass.setInputCloud(m_RawCloud);
  pass.setFilterFieldName("z"); 
  pass.setFilterLimits(0.1, 5);
  pass.setKeepOrganized(true); 

  pcl::ApproximateVoxelGrid<ColorPoint> voxel;
  voxel.setInputCloud(m_FilterCloud);
  voxel.setLeafSize(0.01f, 0.01f, 0.01f); // 1cm
              
  pcl::search::KdTree<ColorPoint>::Ptr tree (new pcl::search::KdTree<ColorPoint>());
  pcl::NormalEstimation<ColorPoint, pcl::Normal> ne;
  ne.setInputCloud(m_FilterCloud);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.03); // 3cm

  pass.filter(*m_FilterCloud);   // uses raw cloud to start
  voxel.filter (*m_FilterCloud); 
}

