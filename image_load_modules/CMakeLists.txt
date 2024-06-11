cmake_minimum_required(VERSION 2.8)
project(image_load_modules)

# To suppress the warning of cereal
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-pragmas")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pragmas -Wno-class-memaccess")

include_directories(${PROJECT_BINARY_DIR})

if(ENABLE_REALSENSE2)
  include_directories(${PROJECT_NAME} ${realsense_INCLUDE_DIR})
endif()

add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/ImageLoading.h
  ${CMAKE_CURRENT_SOURCE_DIR}/ImageLoading.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/DatasetImageLoading.h
  ${CMAKE_CURRENT_SOURCE_DIR}/DatasetImageLoading.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/MonocularCameraImageLoading.h
  ${CMAKE_CURRENT_SOURCE_DIR}/MonocularCameraImageLoading.cc
)

if(ENABLE_REALSENSE2)
  add_library(d435i_camera_image_loading SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/D435iCameraImageLoading.h
    ${CMAKE_CURRENT_SOURCE_DIR}/D435iCameraImageLoading.cc
  )
endif()

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

if(ENABLE_REALSENSE2)
  target_link_libraries(d435i_camera_image_loading PUBLIC ${realsense2_LIBRARY})
  target_link_libraries(${PROJECT_NAME} PUBLIC d435i_camera_image_loading)
endif()

install(TARGETS image_load_modules DESTINATION lib${LIB_SUFFIX})

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${YOLO_PLANAR_SLAM_VERSION})
