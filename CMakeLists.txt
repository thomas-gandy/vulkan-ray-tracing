cmake_minimum_required(VERSION 3.25)
project(VulkanRayTracing)

set(CMAKE_CXX_STANDARD 17)

add_executable(VulkanRayTracing main.cpp vulkanApplication.cpp vulkanApplication.h geometry.h camera.cpp camera.h terrain/terrain.cpp math/noise/perlinNoise.cpp math/noise/fractalNoise.cpp shapes/plane.cpp acceleration/kdTree.cpp geometry.cpp utility/files/ply/plyHelper.c utility/files/ply/plyFileHandler.cpp shapes/triangle.cpp)
include_directories(VulkanRayTracing /usr/local/include)
target_link_libraries(VulkanRayTracing /usr/local/lib/libvulkan.dylib /usr/local/lib/libglfw.3.dylib)