cmake_minimum_required(VERSION 3.16)
project(poselink_gtsam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# OFF keeps GTSAM single-threaded so offline replay is bit-reproducible (design
# Q10). Flip ON only if solve time becomes the bottleneck, accepting
# non-deterministic replay.
option(POSELINK_USE_TBB "Build GTSAM path with TBB multithreading" OFF)
option(POSELINK_BUILD_TESTS "Build the standalone estimator test" ON)

find_package(JNI REQUIRED)
find_package(GTSAM REQUIRED)   # pinned, built from source on the Pi (static, TBB off)

# The estimator, shared by the JNI shim and the test.
add_library(pose_graph_core STATIC pose_graph.cpp)
target_link_libraries(pose_graph_core PUBLIC gtsam gtsam_unstable)
if(NOT POSELINK_USE_TBB)
    target_compile_definitions(pose_graph_core PRIVATE POSELINK_NO_TBB)
endif()
# Homebrew libs (e.g. TBB, when GTSAM is built with it) aren't on the default
# macOS linker path. No-op on the Pi.
if(APPLE)
    target_link_directories(pose_graph_core PUBLIC /opt/homebrew/lib)
endif()

add_library(poselink_gtsam SHARED poselink_gtsam.cpp)
target_include_directories(poselink_gtsam PRIVATE ${JNI_INCLUDE_DIRS})
target_link_libraries(poselink_gtsam PRIVATE pose_graph_core)
set_target_properties(poselink_gtsam PROPERTIES PREFIX "lib")

if(POSELINK_BUILD_TESTS)
    add_executable(pose_graph_test test_pose_graph.cpp)
    target_link_libraries(pose_graph_test PRIVATE pose_graph_core)
endif()
