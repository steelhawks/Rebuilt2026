cmake_minimum_required(VERSION 3.16)
project(poselink_gtsam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# OFF keeps GTSAM single-threaded so offline replay is bit-reproducible (design
# Q10). Flip ON only if solve time becomes the bottleneck, accepting
# non-deterministic replay.
option(POSELINK_USE_TBB "Build GTSAM path with TBB multithreading" OFF)

find_package(JNI REQUIRED)
find_package(GTSAM REQUIRED)   # pinned, built from source on the Pi (static, TBB off)

add_library(poselink_gtsam SHARED
    poselink_gtsam.cpp
    pose_graph.cpp)

target_include_directories(poselink_gtsam PRIVATE ${JNI_INCLUDE_DIRS})
target_link_libraries(poselink_gtsam PRIVATE gtsam gtsam_unstable)

# Homebrew libs (e.g. TBB, when GTSAM is built with it) aren't on the default
# macOS linker path. No-op on the Pi.
if(APPLE)
    target_link_directories(poselink_gtsam PRIVATE /opt/homebrew/lib)
endif()

if(NOT POSELINK_USE_TBB)
    target_compile_definitions(poselink_gtsam PRIVATE POSELINK_NO_TBB)
endif()

set_target_properties(poselink_gtsam PROPERTIES PREFIX "lib")
