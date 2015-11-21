#include "driver_kinect.h"


#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <iostream>


static int kFRAME_WIDTH   = 512;
static int kFRAME_HEIGHT  = 424;
   

void DriverKinect::Update() {

  /**/  
  std::map<std::string, DeviceBundle*>::iterator iter;
  for (iter = m_Devices.begin(); iter != m_Devices.end(); ++iter) {

    libfreenect2::Frame undistorted(kFRAME_WIDTH, kFRAME_HEIGHT, 4);
    libfreenect2::Frame registered(kFRAME_WIDTH, kFRAME_HEIGHT, 4);
    libfreenect2::FrameMap frames;

    // synchronize a new frame set
    DeviceBundle* dvc = iter->second;
    dvc->listener->waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    dvc->registration->apply(rgb, depth, &undistorted, &registered); 

    int h = depth->height;
    int w = depth->width;

    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();

    float max_x = std::numeric_limits<float>::min();
    float max_y = std::numeric_limits<float>::min();
    float max_z = std::numeric_limits<float>::min();
    

    for (size_t r = 0; r < h; r++) {
      for (size_t c = 0; c < w; c++) {
        float x;
        float y;
        float z;
        float frgb;
        dvc->registration->getPointXYZRGB(&undistorted, &registered, r, c, x, y, z, frgb);
        
        if (x > max_x) max_x = x;
        if (x < min_x) min_x = x;
        if (y > max_y) max_y = y;
        if (y < min_y) min_y = y;
        if (z > max_z) max_z = z;
        if (z < min_z) min_z = z;

        if (m_Cloud != NULL) {
          int pt = (r * h) + c;
          m_Cloud->points[pt].x =  x;
          m_Cloud->points[pt].y = -y;
          m_Cloud->points[pt].z =  z;
          m_Cloud->points[pt].rgb = frgb;
        }
      }
    }
    
    /**
    std::cout << "Extents: ";
    std::cout << "\tx:" << min_x << "|" << max_x << "[" << (max_x - min_x) << "]";
    std::cout << "\ty:" << min_y << "|" << max_y << "[" << (max_y - min_y) << "]";
    std::cout << "\tz:" << min_z << "|" << max_z << "[" << (max_z - min_z) << "]";
    std::cout << std::endl;
    **/

    dvc->listener->release(frames);

  }
  /**/
}

DriverKinect::DriverKinect()
  :m_Cloud(NULL) {
}

DriverKinect::~DriverKinect() {

  std::map<std::string, DeviceBundle*>::iterator iter;
  for (iter = m_Devices.begin(); iter != m_Devices.end(); ++iter) {
    
    DeviceBundle* dvc = iter->second;
    dvc->device->stop();
    dvc->device->close();

    delete dvc->listener;
    delete dvc->registration;
    //delete dvc->pipeline; // causes segfault
    delete dvc;
  }
}


ColorPointCloudPtr DriverKinect::Initialize() {

  //libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));

  int numDevices = m_Freenect.enumerateDevices();
  if (numDevices == 0)
  {
    std::cout << "no device connected!" << std::endl;
  } 

  // build our device map and initialize the m_Devices
  for (int i = 0; i < numDevices; i++) {
    
    DeviceBundle* dvcBundle = new DeviceBundle;
    dvcBundle->pipeline = new libfreenect2::OpenCLPacketPipeline();
    dvcBundle->listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    dvcBundle->device = m_Freenect.openDevice(i, dvcBundle->pipeline);
    m_Devices[dvcBundle->device->getSerialNumber()] = dvcBundle;
    
    libfreenect2::Freenect2Device *dev = dvcBundle->device;
    std::cout << "Initializing Device [" << i << "]: " << dev->getSerialNumber() << std::endl;
    dev->setColorFrameListener(dvcBundle->listener);
    dev->setIrAndDepthFrameListener(dvcBundle->listener);
    dev->start();
    dvcBundle->registration = new libfreenect2::Registration(dev->getIrCameraParams(), 
                                                            dev->getColorCameraParams());

  }/**/

  m_Cloud = ColorPointCloudPtr(new ColorPointCloud(kFRAME_WIDTH, kFRAME_HEIGHT));
  m_Cloud->is_dense = false;
  return m_Cloud;
}