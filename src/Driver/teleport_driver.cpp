
#include <iostream>
#include <signal.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "teleport_types.h"
#include "driver_kinect.h"

sig_atomic_t Running = 1;

DriverInterface* interface = NULL;
ColorPointCloudPtr cloud = NULL;
pcl::visualization::PCLVisualizer* viewer = NULL;

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
  viewer->addPointCloud(cloud, "teleport");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "teleport");
}

void Startup() {
  signal(SIGINT, sigint_handler);

  interface = new DriverKinect;
  cloud = interface->Initialize();
  
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
    }

    if (viewer) {
      if (interface) {
        viewer->updatePointCloud(cloud, "teleport");
      }
      viewer->spinOnce(6);
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