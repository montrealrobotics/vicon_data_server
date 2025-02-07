cmake_minimum_required(VERSION 3.10)
project(ViconDataServer)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)

link_directories(lib)

find_package(Threads REQUIRED)

add_executable(ViconDataServer src/ViconDataServer.cpp)

add_subdirectory(external/cppzmq)
include_directories(external/cppzmq)

target_link_libraries(ViconDataServer 
    ${CMAKE_THREAD_LIBS_INIT}
    zmq
    ${CMAKE_SOURCE_DIR}/lib/libViconDataStreamSDK_CPP.so
)

