#include <iostream>
#include <signal.h>
#include <thread>    
#include <mutex>    

#include <gflags/gflags.h>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "asio.hpp"
#include "pointstream_common.h"


using asio::ip::tcp;

enum { max_length = 1024 };

DEFINE_string(host, "localhost", "single resolvable network address to connect to");
DEFINE_string(port, "6969", "port to connect to");
DEFINE_bool(viewer, true, "run the visualizer");

std::atomic<int> Running(1);
std::vector<std::thread> threads;
// Visualizer 
std::unique_ptr<pcl::visualization::PCLVisualizer> viewer = NULL;
const char* kCLOUD_ID = "PointStream";

ColorPointCloudPtr cloud = NULL;   // Filtered cloud received
std::mutex cloud_mutex;     // locking guard for reading/writing the latest point cloud


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
  cloud = ColorPointCloudPtr(new ColorPointCloud(512, 424));
  cloud->is_dense = false;

  viewer = std::unique_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("PointStream Client Visualizer"));
  viewer->addPointCloud(cloud, kCLOUD_ID);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, kCLOUD_ID);
  viewer->initCameraParameters();
  viewer->setShowFPS(true);  
}

//////////////////////////////////////////////////////////////////////////////
/**
 * Set up the Network
 */
 void NetworkLoop() {
  while (Running == 1) {
    //get_io_service().run_one();
  }
}

void InitNetwork() {
  asio::io_service io_service;
  tcp::socket s(io_service);
  tcp::resolver resolver(io_service);
 
  std::cout << "Waiting to connect to " << FLAGS_host << ":" << FLAGS_port << std::endl;
  asio::connect(s, resolver.resolve({FLAGS_host, FLAGS_port}));
  std::cout << "Connected to " << s.remote_endpoint() << std::endl;
  //server = std::unique_ptr<PointStreamServer>(new PointStreamServer(FLAGS_port));
  //server->StartListening();
  //threads.push_back(std::thread(NetworkLoop));
}


//////////////////////////////////////////////////////////////////////////////
/** 
 * Releasing resources and memory
 */
void Shutdown() {
  std::cout << "Client Shutdown." << std::endl;

  if (viewer) {
      viewer->close();
  }
  

  for (auto& th : threads) { th.join(); }
  threads.clear();
}


//////////////////////////////////////////////////////////////////////////////
/**
 * Bootstrapping the processes and memory
 */
void Startup() {
  signal(SIGINT, sigint_handler);
  
  if (FLAGS_viewer)
    InitVisualizer();
  
  InitNetwork();
}

//////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  try
  {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // Bootstrap. Including spawing threads
    Startup(); 

    // On the main thread, update the visualizer
    while (Running == 1) {
      if (viewer) {
        std::lock_guard<std::mutex> lock(cloud_mutex);
        viewer->updatePointCloud(cloud, kCLOUD_ID);
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