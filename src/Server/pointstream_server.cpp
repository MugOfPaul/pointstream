
#include <iostream>
#include <signal.h>
#include <thread>    
#include <mutex>    

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include "pointstream_types.h"
#include "pointstream_servernet.h"
#include "driver_kinect.h"

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
std::unique_ptr<PointStreamServer> server = NULL; // Primary interface for handling incoming connections 

// Point Cloud Source device
std::unique_ptr<DriverInterface> interface = NULL; // Our interface to whatever sensing device providing the point cloud
ColorPointCloudPtr raw_cloud = NULL;      // Raw cloud from the interface
ColorPointCloudPtr filter_cloud = NULL;   // Filtered cloud to be sent out
NormalCloudPtr cloud_normals = NULL;      // estimated normals
std::mutex filter_cloud_mutex;            // locking guard for reading/writing the latest point cloud
    
// Visualizer 
std::unique_ptr<pcl::visualization::PCLVisualizer> viewer = NULL;
const char* kCLOUD_ID = "PointStream";

//////////////////////////////////////////////////////////////////////////////
/**
 * Our Ctrl-C handler
 **/
void sigint_handler(int s) { Running = 0; }


//////////////////////////////////////////////////////////////////////////////
// Helper function to pack a 32-bit color
inline float pack_color(uint8_t r, uint8_t g, uint8_t b) {
  int32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  return *reinterpret_cast<float*>(&rgb);
}


//////////////////////////////////////////////////////////////////////////////
/**
 * Set up the Visualizer
 */
void InitVisualizer() {
  viewer = std::unique_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("PointStream Driver Visualizer"));
  viewer->addPointCloud(filter_cloud, kCLOUD_ID);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, kCLOUD_ID);
  viewer->initCameraParameters();
  viewer->setShowFPS(true);  
}


//////////////////////////////////////////////////////////////////////////////
/** 
 * Our main loop for getting a raw point cloud and filtering it
 */
void DriverInterfaceLoop() {

  // Master filters
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
        
        std::lock_guard<std::mutex> lock(filter_cloud_mutex);

        pass.filter(*filter_cloud); 
        voxel.filter (*filter_cloud);

        //std::cout << "Cloud Points: " << raw_cloud->size() << "\t Filtered: " << filter_cloud->size() << std::endl;
        //ne.compute (*cloud_normals);
    }
  }
}



//////////////////////////////////////////////////////////////////////////////
/**
 * Set up the Network
 */
 void NetworkLoop() {
  while (Running == 1) {
    if (server) {
      server->Update();
    }
  }
}

void InitNetwork() {
  server = std::unique_ptr<PointStreamServer>(new PointStreamServer(6969));
  server->StartListening();
  threads.push_back(std::thread(NetworkLoop));
}

//////////////////////////////////////////////////////////////////////////////
void InitInterface() {
  
  interface = std::unique_ptr<DriverKinect>(new DriverKinect());

  if (interface) {
    raw_cloud = interface->Initialize();
    filter_cloud = ColorPointCloudPtr(new ColorPointCloud);
    cloud_normals = NormalCloudPtr(new NormalCloud);

    InitVisualizer();
    threads.push_back(std::thread(DriverInterfaceLoop));
  }
}


//////////////////////////////////////////////////////////////////////////////
/**
 * Bootstrapping the processes and memory
 */
void Startup() {
  signal(SIGINT, sigint_handler);
  //InitInterface();
  InitNetwork();
}


//////////////////////////////////////////////////////////////////////////////
/** 
 * Releasing resources and memory
 */
void Shutdown() {
  std::cout << "Driver Shutdown." << std::endl;

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

  // Bootstrap. Including spawing threads
  Startup(); 

  // On the main thread, update the visualizer
  while (Running == 1) {
    if (viewer) {
      std::lock_guard<std::mutex> lock(filter_cloud_mutex);
      viewer->updatePointCloud(filter_cloud, kCLOUD_ID);
      viewer->spinOnce(12);
    } else {
      sleep(1);
    }
  }

  // We got a kill signal of some sort, so clean everything up.
  Shutdown();  
  return 0;
}