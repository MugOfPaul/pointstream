#include <iostream>
#include <signal.h>
#include <thread>    
#include <mutex>    

#include <gflags/gflags.h>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "asio.hpp"
#include "pointstream_common.h"
#include "pointstream_consumer.h"


using asio::ip::tcp;

enum { max_length = 1024 };

DEFINE_string(host, "localhost", "single resolvable network address to connect to");
DEFINE_int32(port, 6969, "port to connect to");
DEFINE_bool(viewer, false, "run the visualizer");

std::atomic<int> Running(1);
std::vector<std::thread> threads;

// Networking
std::unique_ptr<PointStreamConsumer> consumer = nullptr; 

// Visualizer 
std::unique_ptr<pcl::visualization::PCLVisualizer> viewer = NULL;
const char* kCloudId = "PointStream";

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
  viewer->addPointCloud(cloud, kCloudId);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, kCloudId);
  viewer->initCameraParameters();
  viewer->setShowFPS(true);  
}

//////////////////////////////////////////////////////////////////////////////
/**
 * Set up the Network
 */
 void NetworkLoop() {
  consumer = std::unique_ptr<PointStreamConsumer>(new PointStreamConsumer());
  consumer->Start(FLAGS_host, FLAGS_port);

  while (Running == 1) {
      consumer->Update();
  }

  consumer->Stop();
}

void InitNetwork() {
  threads.push_back(std::thread(NetworkLoop));
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
        
          if (consumer != nullptr) {
            std::lock_guard<std::mutex> lock(cloud_mutex);

            // Copy the points from the network side
            PointStreamPointBuffer temp(consumer->GetPoints());
          
            if (temp.size() > 0) {
              cloud->clear();

              for (PointStreamPointBuffer::iterator iter = temp.begin(); iter != temp.end(); ++iter) {
                ColorPoint pcl_point;
                pcl_point.x   = iter->x;
                pcl_point.y   = iter->y;
                pcl_point.z   = iter->z;
                pcl_point.rgb = iter->rgb;
                cloud->push_back(pcl_point);
              }
              viewer->updatePointCloud(cloud, kCloudId);
            }
          }
          
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