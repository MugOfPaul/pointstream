#pragma once

#include "device_interface.h"

#include <string>
#include <map>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>


/**
 * This represents a connection to a single device
 */
struct DeviceBundle {
  libfreenect2::Freenect2Device* device;
  libfreenect2::PacketPipeline* pipeline;
  libfreenect2::SyncMultiFrameListener* listener;
  libfreenect2::Registration* registration;
};

class DeviceKinect : public DeviceInterface {
  public:
    DeviceKinect(std::shared_ptr<PointStreamProcessor> proc);
    virtual ~DeviceKinect();
    void Initialize();
    void Update();

  private:
    libfreenect2::Freenect2 freenect;
    std::map<std::string,DeviceBundle*> devices;
};