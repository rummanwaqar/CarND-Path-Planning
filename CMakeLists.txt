project(Path_Planning)
cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

include_directories(include/ include/Eigen-3.3/)

add_executable(path_planning src/main.cpp src/motion.cpp src/io.cpp src/helpers.cpp)
target_link_libraries(path_planning z ssl uv uWS)