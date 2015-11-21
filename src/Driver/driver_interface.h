#pragma once

#include "teleport_types.h"

class DriverInterface {
  public:
    virtual ~DriverInterface() { }
    virtual ColorPointCloudPtr Initialize() = 0;
    virtual void Update() = 0;
    virtual ColorPointCloudPtr PointCloud() = 0;
};
