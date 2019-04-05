cmake_minimum_required(VERSION 2.6)
project(openedge)

# Set(EIGEN3_INCLUDE_DIR "/usr/local/include/eigen3")
# include_directories("${EIGEN3_INCLUDE_DIR}")
# 
# find_package(OpenMVG REQUIRED)
# include_directories(${OPENMVG_INCLUDE_DIRS})

add_executable(openedge main.cpp)

install(TARGETS openedge RUNTIME DESTINATION bin)

add_subdirectory(src)