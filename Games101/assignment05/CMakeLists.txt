cmake_minimum_required(VERSION 3.10)
project(RayTracing)

find_package(Eigen3 REQUIRED)
include_directories("C:\\Program Files (x86)\\Eigen3\\include\\")

set(CMAKE_CXX_STANDARD 17)

add_executable(RayTracing main.cpp Object.hpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp Scene.hpp Light.hpp Renderer.cpp)
target_compile_options(RayTracing PUBLIC -Wall -Wextra -pedantic -Wshadow -Wreturn-type )
target_compile_features(RayTracing PUBLIC cxx_std_17)

