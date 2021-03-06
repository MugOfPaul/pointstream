set(_target PointStreamServer)


find_package(PkgConfig)    # try find PKGConfig as it will be used if found
find_package(freenect2 CONFIG REQUIRED)
find_package(PCL 1.7.2 REQUIRED)
find_package(gflags REQUIRED)
find_library(GFLAGS_LIBRARY gflags)

add_definitions(${PCL_DEFINITIONS})

set(_target_includes 
  .
  ..
  ${freenect2_INCLUDE_DIR}
  ${asio_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

set(_target_libs 
  ${GFLAGS_LIBRARY}
  ${freenect2_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_GEOMETRY_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  ${PCL_FEATURES_LIBRARIES}
)

set(_target_srcs 
  ../pointstream_common.cpp
  server.cpp
  pointstream_processor.cpp
  device_kinect.cpp
  pointstream_server.cpp
)

add_executable(${_target} ${_target_srcs})
target_include_directories(${_target} PRIVATE ${_target_includes})
target_link_libraries(${_target} ${_target_libs})
