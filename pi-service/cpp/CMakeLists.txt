cmake_minimum_required(VERSION 3.16)
project(poselink_gtsam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Determinism vs multicore (see design Q10): OFF keeps GTSAM single-threaded so
# offline replay is bit-reproducible. Flip ON only if solve time becomes the
# bottleneck, accepting non-deterministic replay.
option(POSELINK_USE_TBB "Build GTSAM path with TBB multithreading" OFF)

find_package(JNI REQUIRED)

add_library(poselink_gtsam SHARED poselink_gtsam.cpp)
target_include_directories(poselink_gtsam PRIVATE ${JNI_INCLUDE_DIRS})

# --- Slice 2 wiring (kept here as the seam; enable once GTSAM is built on the Pi) ---
# find_package(GTSAM REQUIRED)                 # pinned, built from source, static
# target_link_libraries(poselink_gtsam PRIVATE gtsam)
# if(NOT POSELINK_USE_TBB)
#   target_compile_definitions(poselink_gtsam PRIVATE POSELINK_NO_TBB)
# endif()

# Output name -> libposelink_gtsam.so (System.loadLibrary("poselink_gtsam")).
set_target_properties(poselink_gtsam PROPERTIES PREFIX "lib")
