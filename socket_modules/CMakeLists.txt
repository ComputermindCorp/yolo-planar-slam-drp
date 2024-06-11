cmake_minimum_required(VERSION 2.8)
project(socket_modules)

# To suppress the warning of cereal
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-pragmas")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pragmas -Wno-class-memaccess")

include_directories(${PROJECT_BINARY_DIR})

if(ENABLE_DRP)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../drp_modules)
endif()

if(NOT PROTOBUF_PROTOC_EXECUTABLE)
    message(FATAL_ERROR "Could not find protoc executable (PROTOBUF_PROTOC_EXECUTABLE)")
endif()
message(STATUS "Found protoc executable: ${PROTOBUF_PROTOC_EXECUTABLE}")

# ----- Protobuf transpile -----

protobuf_generate_cpp(MAP_PB_SOURCE MAP_PB_HEADER protobuf/map_segment.proto)
set_source_files_properties(${MAP_PB_HEADER} ${MAP_PB_SOURCE}
                            COMPILE_FLAGS -Wno-unused-variable)

add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/DataSerializer.h
  ${CMAKE_CURRENT_SOURCE_DIR}/SocketClient.h
  ${CMAKE_CURRENT_SOURCE_DIR}/SocketViewer.h
  ${CMAKE_CURRENT_SOURCE_DIR}/DataSerializer.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/SocketClient.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/SocketViewer.cc
  ${MAP_PB_SOURCE}
)

set_source_files_properties(${MAP_PB_SOURCE} PROPERTIES
                            COMPILE_FLAGS -Wno-unused-parameter)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${SIOCLIENT_LIBRARY}
  ${PROTOBUF_LIBRARY}
  ${PROTOBUF_LIBRARIES}
)

install(TARGETS socket_modules DESTINATION lib${LIB_SUFFIX})

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${YOLO_PLANAR_SLAM_VERSION})
