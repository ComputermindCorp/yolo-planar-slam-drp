cmake_minimum_required(VERSION 2.8)
project(image_proc_modules)

# To suppress the warning of cereal
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-pragmas")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pragmas -Wno-class-memaccess")

include_directories(${PROJECT_BINARY_DIR})

add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/ImageProcessing.h
  ${CMAKE_CURRENT_SOURCE_DIR}/ImageProcessing.cc
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  drp_modules
)

install(TARGETS image_proc_modules DESTINATION lib${LIB_SUFFIX})

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${YOLO_PLANAR_SLAM_VERSION})
