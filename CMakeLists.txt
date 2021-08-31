# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

set(CMAKE_C_COMPILER "clang-10")
set(CMAKE_CXX_COMPILER "/usr/bin/clang++-10")

project(RecirculationSurfaces VERSION 0.1 LANGUAGES CXX)

# Set the path where cmake modules are stored
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)
message("Module path: " ${CMAKE_MODULE_PATH})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#export compilation database to use with auto complete
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

# search for libraries that are needed: Flann and LZ4
find_package(LZ4 REQUIRED)

if (LZ4_FOUND)
  message("LZ4 include directory: " STATUS ${LZ4_INCLUDE_DIR})
else(LZ4_FOUND)
  message("ERROR: LZ4 could not be found.")
endif()

find_package(Flann REQUIRED)
if (FLANN_FOUND)
  message("Flann include directory:" STATUS ${FLANN_INCLUDE_DIR})
  message("Flann libraries:" STATUS ${FLANN_LIBRARIES})
else(FLANN_FOUND)
  message("ERROR: Flann could not be found.")
endif()

# Define an executable
add_executable(recsurface
               src/main.cpp
               src/aabb.cpp
               src/amiradataset.cpp
               src/colormap.cpp
               src/critextractor.cpp
               src/doublegyre3D.cpp
               src/globals.cpp
               src/hyperline.cpp
               src/hyperpoint.cpp
               src/math.cpp
               src/perspectivecamera.cpp
               src/progresssaver.cpp
               src/ray.cpp
               src/raytracer.cpp
               src/refraytracer.cpp
               src/recsurface.cpp
               src/scene.cpp
               src/shader.cpp
               src/texture.cpp
               src/timer.cpp
               src/vectorcuboid.cpp
               vclibs/math/rk43.cc
               vclibs/math/ode.cc)

target_include_directories(
  recsurface
    PUBLIC ${FLANN_INCLUDE_DIRS}
    PUBLIC ${LZ4_INCLUDE_DIR}
    PRIVATE ./inc
    PRIVATE ./)

# dependencies for libraries
target_link_libraries(
  recsurface
    PUBLIC ${FLANN_LIBRARIES}
    PUBLIC ${LZ4_LIBRARY}
    PRIVATE stdc++fs)

# additional compiler flags for more warnings
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -fopenmp")
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -fopenmp")   # -llz4
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fopenmp") # -llz4