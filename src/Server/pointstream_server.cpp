
#include <iostream>
#include <signal.h>
#include <thread>    
#include <mutex>    
#include <gflags/gflags.h>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include "pointstream_common.h"
#include "pointstream_processor.h"
#include "pointstream_servernet.h"
#include "device_kinect.h"

DEFINE_bool(net, true, "should run network server");
DEFINE_int32(port, 6969, "port to listen on");
DEFINE_bool(viewer, false, "run the visualizer");

/** Notebook
 *
 * X,Y,Z,RGB,nX,nY,nZ,id == 8 dwords == 64 bytes == 512 bits
 * K4W2 raw point cloud: 217088 filtered: <150000
 *
 * Total Frame: 
 *      Raw: 111149056 bits   (111150kbits) (111mbits)
 *            13893632 bytes      (13568KB)  (13.25MB)
 *
 *      Naive Filtered: 76800000 bits   (76800kbits)  (76.8mbits)
 *                       9600000 bytes      (9375KB)     (9.15MB)
 *
 * Typical internet data rate of 10mbps (up & down)
 *      Filtered: 1fps 
 *      Raw: <1fps
 */


//////////////////////////////////////////////////////////////////////////////
 /**
  * Module vars/globals
  */
std::atomic<int> Running(1);
std::vector<std::thread> threads;

// Networking
std::unique_ptr<PointStreamServer> server = nullptr; // Primary interface for handling incoming connections 

// Our actual processor
std::shared_ptr<PointStreamProcessor> processor = nullptr;

// Point Cloud Source device
std::unique_ptr<DeviceInterface> interface = nullptr; // Our interface to whatever sensing device providing the point cloud

// Visualizer 
std::unique_ptr<pcl::visualization::PCLVisualizer> viewer = nullptr;
const char* kCLOUD_ID = "PointStream";

//////////////////////////////////////////////////////////////////////////////
/**
 * Our Ctrl-C handler
 **/
void sigint_handler(int s) { Running = 0; }



//////////////////////////////////////////////////////////////////////////////
/**
 * Set up the Visualizer
 */
void InitVisualizer() {
  viewer = std::unique_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("PointStream Server Visualizer"));
  viewer->addPointCloud(processor->GetLatestCloudFiltered(), kCLOUD_ID);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, kCLOUD_ID);
  viewer->initCameraParameters();
  viewer->setShowFPS(true);  
}


//////////////////////////////////////////////////////////////////////////////
/** 
 * Our main loop for getting a raw point cloud from a device and processing it
 */
void DeviceInterfaceLoop() {
  while (Running == 1) {
    if (interface) {
        interface->Update();   
    }
  }
}



//////////////////////////////////////////////////////////////////////////////
/**
 * Update for listeners and transmit point cloud
 */
 void NetworkLoop() {
  while (Running == 1) {
    if (server) {
      server->Update();
    }
  }
}

void InitNetwork() {
  server = std::unique_ptr<PointStreamServer>(new PointStreamServer(FLAGS_port));
  server->StartListening();
  threads.push_back(std::thread(NetworkLoop));
}

//////////////////////////////////////////////////////////////////////////////
void InitInterface() {
  
  processor = std::make_shared<PointStreamProcessor>();
  interface = std::unique_ptr<DeviceKinect>(new DeviceKinect(processor));

  if (interface) {
    interface->Initialize();

    if (FLAGS_viewer)
       InitVisualizer();
      
      threads.push_back(std::thread(DeviceInterfaceLoop));
  }
}


//////////////////////////////////////////////////////////////////////////////
/**
 * Bootstrapping the processes and memory
 */
void Startup() {
  signal(SIGINT, sigint_handler);
  
  InitInterface();
  
  if (FLAGS_net)
    InitNetwork();
}


//////////////////////////////////////////////////////////////////////////////
/** 
 * Releasing resources and memory
 */
void Shutdown() {
  std::cout << "Full Shutdown." << std::endl;

  if (server) {
    server->Stop();
  }

  if (interface) {
    // shut the visualizer down if we have one
    if (viewer) {
      viewer->close();
    }
  }

  for (auto& th : threads) { th.join(); }
  threads.clear();
}



//////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  try
  {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // Bootstrap. Including spawing threads
    Startup(); 

    // On the main thread, update the visualizer
    while (Running == 1) {
      if (viewer) {
        viewer->updatePointCloud(processor->GetLatestCloudFiltered(), kCLOUD_ID);
        viewer->spinOnce(12);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
      }
    }

  } catch (std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  Shutdown();  
  return 0;
}