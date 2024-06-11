cmake_minimum_required(VERSION 2.8)
project(drp_ai_modules)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-int-to-pointer-cast")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-int-to-pointer-cast")

# To suppress the warning of cereal
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-pragmas")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pragmas -Wno-class-memaccess")

if(NOT ENABLE_DRP)
  message(FATAL "ENABLE_DRP must be ON.")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

if(ENABLE_TVM)
  add_subdirectory(tvm)
endif()

if(NOT ENABLE_YOCTO AND (ENABLE_DRP OR ENABLE_DRP_AI))
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../v2x)
endif()

add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/drp/drp_proc.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/recognize/common/box.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/recognize/yoloxs_sp70/yoloxs_sp70_model.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/recognize/yoloxs_dense/yoloxs_dense_model.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/recognize/recognize_base.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/YoloDetector_drp.h
  ${CMAKE_CURRENT_SOURCE_DIR}/YoloDetector_drp.cc
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../Thirdparty/DBoW2/lib/libDBoW2.so
  ${CMAKE_CURRENT_SOURCE_DIR}/../Thirdparty/g2o/lib/libg2o.so
  drp_modules
)

if(ENABLE_TVM)
  target_link_libraries(${PROJECT_NAME} tvm)
endif()

install(TARGETS drp_ai_modules DESTINATION lib${LIB_SUFFIX})

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${YOLO_PLANAR_SLAM_VERSION})
