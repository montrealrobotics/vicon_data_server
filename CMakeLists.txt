cmake_minimum_required(VERSION 3.10)
project(ViconDataServer)

set(CMAKE_CXX_STANDARD 14)

# Find ZeroMQ
find_package(Threads REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Ensure cppzmq is available
add_subdirectory(external/cppzmq)
include_directories(external/cppzmq)

# Include project headers
include_directories(include)

# Link directories
link_directories(lib)

# Add the executable
add_executable(ViconDataServer src/ViconDataServer.cpp)

# Link against Vicon SDK, cppzmq, and ZeroMQ
target_link_libraries(ViconDataServer 
    ${CMAKE_THREAD_LIBS_INIT}
    zmq
    cppzmq
    ${CMAKE_SOURCE_DIR}/lib/libViconDataStreamSDK_CPP.so
)

