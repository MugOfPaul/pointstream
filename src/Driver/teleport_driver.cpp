
#include <iostream>
#include <signal.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "teleport_types.h"
#include "driver_kinect.h"

sig_atomic_t Running = 1;

DriverInterface* interface = NULL;
ColorPointCloudPtr raw_cloud = NULL;
ColorPointCloudPtr filter_cloud = NULL;
pcl::visualization::PCLVisualizer* viewer = NULL;

pcl::PassThrough<ColorPoint> passthroughFilter;

void sigint_handler(int s) { Running = 0; }

inline float pack_color(uint8_t r, uint8_t g, uint8_t b) {
  int32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  return *reinterpret_cast<float*>(&rgb);
}

void InitVisualizer() {
  viewer = new pcl::visualization::PCLVisualizer("Teleport Driver Visualizer");
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters();
  viewer->setShowFPS(true);  
  //viewer->setCameraPosition(0.0, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  viewer->addPointCloud(filter_cloud, "teleport");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "teleport");
}

void Startup() {
  signal(SIGINT, sigint_handler);

  interface = new DriverKinect;
  raw_cloud = interface->Initialize();
  filter_cloud = ColorPointCloudPtr(new ColorPointCloud);

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
  while (Running == 1) {
    if (interface) {
        interface->Update();   
        
        
        pcl::PassThrough<ColorPoint> pass;
        pass.setInputCloud(raw_cloud);
        pass.setFilterFieldName("z"); // filter z dimension
        pass.setFilterLimits(0.1, 5);
        pass.setKeepOrganized(true); // Important: to keep the "image-structure" after filtering, if not it becomes 1D (and sparse)
        pass.filter(*filter_cloud); 

        pcl::ApproximateVoxelGrid<ColorPoint> voxel;
        voxel.setInputCloud (filter_cloud);
        voxel.setLeafSize (0.001f, 0.001f, 0.1f);
        voxel.filter (*filter_cloud);

        /*
        pcl::StatisticalOutlierRemoval<ColorPoint> noise;
        noise.setInputCloud (filter_cloud);
        noise.setMeanK(50);
        noise.setStddevMulThresh (1.0);
        noise.filter (*filter_cloud);
        **/
    }

    if (viewer) {
      viewer->updatePointCloud(filter_cloud, "teleport");
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