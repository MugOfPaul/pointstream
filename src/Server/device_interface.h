#pragma once

#include "pointstream_common.h"
#include "pointstream_processor.h"

class DeviceInterface {
  public:
    DeviceInterface(std::shared_ptr<PointStreamProcessor> proc):processor(proc) { }
    virtual ~DeviceInterface() { processor = nullptr; }
    virtual void Initialize() = 0;
    virtual void Update() = 0;

  protected:
    std::shared_ptr<PointStreamProcessor> processor;
};
