#pragma once

#include "driver_interface.h"

#include <string>
#include <map>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>


struct DeviceBundle {
  libfreenect2::Freenect2Device* device;
  libfreenect2::PacketPipeline* pipeline;
  libfreenect2::SyncMultiFrameListener* listener;
  libfreenect2::Registration* registration;
};

class DriverKinect : public DriverInterface {
  public:
    DriverKinect();
    virtual ~DriverKinect();
    ColorPointCloudPtr Initialize();
    void Update();
    ColorPointCloudPtr PointCloud() { return m_Cloud; }

  private:
    libfreenect2::Freenect2 m_Freenect;
    std::map<std::string,DeviceBundle*> m_Devices;
    ColorPointCloudPtr m_Cloud;
};