#pragma once
#include "pointstream_common.h"

class PointStreamProcessor {
public:
  PointStreamProcessor();
  virtual ~PointStreamProcessor();

  void Initialize(short width, short height);
  ColorPointCloudPtr GetLatestCloudFiltered();

  void BeginFrame();
  void SubmitPoint(int index, float x, float y, float z, float rgb);
  void EndFrame();

private:
  void RunFilters();
  ColorPointCloudPtr m_RawCloud = NULL;      
  ColorPointCloudPtr m_FilterCloud = NULL;  
  std::mutex filter_cloud_mutex;

};