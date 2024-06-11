cmake_minimum_required(VERSION 2.8)
project(drp_modules)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-int-to-pointer-cast")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-int-to-pointer-cast")

# To suppress the warning of cereal
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-pragmas")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pragmas -Wno-class-memaccess")

if(NOT ENABLE_YOCTO AND (ENABLE_DRP OR ENABLE_DRP_AI))
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../v2x)
endif()

add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/ORBextractor_drp.h
  ${CMAKE_CURRENT_SOURCE_DIR}/Frame_drp.h
  ${CMAKE_CURRENT_SOURCE_DIR}/drp.h
  ${CMAKE_CURRENT_SOURCE_DIR}/ORBextractor_drp.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/Frame_drp.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/drp.cc
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  DBoW2
  g2o
)

install(TARGETS drp_modules DESTINATION lib${LIB_SUFFIX})

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${YOLO_PLANAR_SLAM_VERSION})
