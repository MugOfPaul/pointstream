#pragma once

#include "pointstream_common.h"
#include "pointstream_processor.h"

class DeviceInterface {
  public:
    DeviceInterface(std::shared_ptr<PointStreamProcessor> proc):m_Processor(proc) { }
    virtual ~DeviceInterface() { m_Processor = nullptr; }
    virtual void Initialize() = 0;
    virtual void Update() = 0;

  protected:
    std::shared_ptr<PointStreamProcessor> m_Processor;
};
