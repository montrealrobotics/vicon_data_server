cmake_minimum_required(VERSION 3.10)
project(ViconDataServer)

set(CMAKE_CXX_STANDARD 14)

# Find ZeroMQ
find_package(Threads REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(external/cppzmq)
include_directories(external/cppzmq)

include_directories(include)

link_directories(lib)

add_executable(ViconDataServer src/ViconDataServer.cpp)

target_link_libraries(ViconDataServer 
    ${CMAKE_THREAD_LIBS_INIT}
    zmq
    cppzmq
    ${CMAKE_SOURCE_DIR}/lib/libViconDataStreamSDK_CPP.so
)

