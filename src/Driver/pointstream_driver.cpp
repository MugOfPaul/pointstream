
#include <iostream>
#include <signal.h>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <asio.hpp>

#include "pointstream_types.h"
#include "driver_kinect.h"

/** Notebook
 *
 * X,Y,Z,RGB,nX,nY,nZ,id == 8 dwords == 64 bytes == 512 bits
 * K4W2 raw point cloud: 217088 filtered: <150000
 *
 * Total Frame: 
 *      Raw: 111149056 bits   (111150kbits) (111mbits)
 *            13893632 bytes  (13568KB) (13.25MB)
 *
 *      Naive Filtered: 76800000 bits (76800kbits) (76.8mbits)
 *            9600000 bytes  (9375KB) (9.15MB)
 *
 * Typical internet data rate of 10mbps (up & down)
 *      Filtered: 1fps Raw: <1fps
 */

sig_atomic_t Running = 1;

DriverInterface* interface = NULL;
ColorPointCloudPtr raw_cloud = NULL;
ColorPointCloudPtr filter_cloud = NULL;
NormalCloudPtr cloud_normals = NULL;
#define kCLOUD_ID "PointStream"

pcl::visualization::PCLVisualizer* viewer = NULL;

pcl::PassThrough<ColorPoint> passthroughFilter;

void sigint_handler(int s) { Running = 0; }

inline float pack_color(uint8_t r, uint8_t g, uint8_t b) {
  int32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  return *reinterpret_cast<float*>(&rgb);
}

void InitVisualizer() {
  viewer = new pcl::visualization::PCLVisualizer("PointStream Driver Visualizer");
  //viewer->addCoordinateSystem (1.0);
  //viewer->setCameraPosition(0.0, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  viewer->addPointCloud(filter_cloud, kCLOUD_ID);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, kCLOUD_ID);

  //viewer->addPointCloudNormals<ColorPoint, Normal>(filter_cloud, cloud_normals, 100, 0.01, "normals");
  //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, kCLOUD_ID); 
  viewer->initCameraParameters();
  viewer->setShowFPS(true);  
}

void Startup() {
  signal(SIGINT, sigint_handler);

  interface = new DriverKinect;
  raw_cloud = interface->Initialize();

  filter_cloud = ColorPointCloudPtr(new ColorPointCloud);
  cloud_normals = NormalCloudPtr(new NormalCloud);

  InitVisualizer();
}

void Shutdown() {
  std::cout << "Driver Shutdown." << std::endl;
  if (viewer) {
    viewer->close();
    delete viewer;
  }

  if (interface)
    delete interface;
}

void Loop() {

  pcl::PassThrough<ColorPoint> pass;
  pass.setInputCloud(raw_cloud);
  pass.setFilterFieldName("z"); 
  pass.setFilterLimits(0.1, 5);
  pass.setKeepOrganized(true); 

  pcl::ApproximateVoxelGrid<ColorPoint> voxel;
  voxel.setInputCloud (filter_cloud);
  voxel.setLeafSize (0.01f, 0.01f, 0.01f); // 1cm
              
  pcl::search::KdTree<ColorPoint>::Ptr tree (new pcl::search::KdTree<ColorPoint>());
  pcl::NormalEstimation<ColorPoint, pcl::Normal> ne;
  ne.setInputCloud(filter_cloud);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.03); // 3cm
  

  while (Running == 1) {
    if (interface) {
        interface->Update();   
        
        pass.filter(*filter_cloud); 
        voxel.filter (*filter_cloud);

        std::cout << "Cloud Points: " << raw_cloud->size() << "\t Filtered: " << filter_cloud->size() << std::endl;
        ne.compute (*cloud_normals);
    }

    if (viewer) {
      viewer->updatePointCloud(filter_cloud, kCLOUD_ID);
      viewer->spinOnce(12);
    }
  }
}

int main(int argc, char *argv[])
{
  Startup();
  Loop();
  Shutdown();  
  return 0;
}