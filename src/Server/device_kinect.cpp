#include "device_kinect.h"


#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <iostream>


static short kFRAME_WIDTH   = 512;
static short kFRAME_HEIGHT  = 424;
   

//////////////////////////////////////////////////////////////////////////////
/** 
 * Primary update loop
 */

void DeviceKinect::Update() {

  processor->BeginFrame();

  // For each connected device, sync and register the frames
  std::map<std::string, DeviceBundle*>::iterator iter;
  for (iter = devices.begin(); iter != devices.end(); ++iter) {

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

    // set up extents
    float max = std::numeric_limits<float>::max(); 
    float min = std::numeric_limits<float>::min();
    float min_x = max; float min_y = max; float min_z = max;
    float max_x = min; float max_y = min; float max_z = min;
    
    // grab each point from the registered frame, update the extents and push to the cloud
    for (size_t r = 0; r < h; r++) {
      for (size_t c = 0; c < w; c++) {
        float x, y, z, frgb;
        dvc->registration->getPointXYZRGB(&undistorted, &registered, r, c, x, y, z, frgb);
        
        // check for extents
        if (x > max_x) max_x = x;
        if (x < min_x) min_x = x;
        if (y > max_y) max_y = y;
        if (y < min_y) min_y = y;
        if (z > max_z) max_z = z;
        if (z < min_z) min_z = z;

        int pt = (r * h) + c;
        processor->SubmitPoint(pt, x, -y, z, frgb);
      }
    }
    dvc->listener->release(frames);
  }

  processor->EndFrame();
  /**/
}


//////////////////////////////////////////////////////////////////////////////
DeviceKinect::DeviceKinect(std::shared_ptr<PointStreamProcessor> proc)
  :DeviceInterface(proc)
{
}


//////////////////////////////////////////////////////////////////////////////
DeviceKinect::~DeviceKinect() {

  // clean up all of our resources
  std::map<std::string, DeviceBundle*>::iterator iter;
  for (iter = devices.begin(); iter != devices.end(); ++iter) {
    
    DeviceBundle* dvc = iter->second;
    dvc->device->stop();
    dvc->device->close();

    delete dvc->listener;
    delete dvc->registration;
    delete dvc->pipeline; // causes segfault
    delete dvc;
  }
}


//////////////////////////////////////////////////////////////////////////////
/** 
 * Connect to attached devices and set up the appropriate point cloud to be 
 * updated every time the update is called
 */
void DeviceKinect::Initialize() {

  //libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));

  int numDevices = freenect.enumerateDevices();
  if (numDevices == 0)
  {
    std::cout << "no device connected!" << std::endl;
    return;
  } 

  // build our device map and initialize the devices
  for (int i = 0; i < numDevices; i++) {
    
    DeviceBundle* dvcBundle = new DeviceBundle;
    dvcBundle->pipeline = new libfreenect2::OpenCLPacketPipeline();
    dvcBundle->listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    dvcBundle->device = freenect.openDevice(i, dvcBundle->pipeline);
    devices[dvcBundle->device->getSerialNumber()] = dvcBundle;
    
    libfreenect2::Freenect2Device *dev = dvcBundle->device;
    std::cout << "Initializing Device [" << i << "]: " << dev->getSerialNumber() << std::endl;
    dev->setColorFrameListener(dvcBundle->listener);
    dev->setIrAndDepthFrameListener(dvcBundle->listener);
    dev->start();
    dvcBundle->registration = new libfreenect2::Registration(dev->getIrCameraParams(), 
                                                            dev->getColorCameraParams());

  }/**/

  processor->Initialize(kFRAME_WIDTH, kFRAME_HEIGHT);
}