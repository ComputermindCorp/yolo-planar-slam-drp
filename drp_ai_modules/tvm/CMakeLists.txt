cmake_minimum_required(VERSION 2.8)
project(tvm)

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

set(TVM_ROOT $ENV{TVM_HOME})
include_directories(${TVM_ROOT}/include)
include_directories(${TVM_ROOT}/3rdparty/dlpack/include)
include_directories(${TVM_ROOT}/3rdparty/dmlc-core/include)
include_directories(${TVM_ROOT}/3rdparty/compiler-rt)

add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/MeraDrpRuntimeWrapper.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/PreRuntimeOcv.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/YoloDetector_tvm.cpp
)

target_link_libraries(${PROJECT_NAME} ${TVM_ROOT}/../obj/build_runtime/V2L/libtvm_runtime.so)

install(TARGETS tvm DESTINATION lib${LIB_SUFFIX})

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${YOLO_PLANAR_SLAM_VERSION})
