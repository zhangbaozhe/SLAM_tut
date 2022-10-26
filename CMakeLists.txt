cmake_minimum_required(VERSION 3.16)
project(SLAM_tut)

set(CMAKE_CXX_STANDARD 11)

#if (WIN32)
#    include_directories(GLEW_INCLUDE_DIR
#        ${PROJECT_SOURCE_DIR}/Dependencies/glew-2.1.0-win32/glew-2.1.0/include)
#    link_directories(GLEW_LIBRARY
#        ${PROJECT_SOURCE_DIR}/Dependencies/glew-2.1.0-win32/glew-2.1.0/lib/Release/x64)
##    add_subdirectory(${PROJECT_SOURCE_DIR}/Dependencies/eigen-3.4.0)
#    add_subdirectory(${PROJECT_SOURCE_DIR}/Dependencies/gflags)
#    add_subdirectory(${PROJECT_SOURCE_DIR}/Dependencies/Pangolin)
#else()
#endif()

find_package(Eigen3 CONFIG REQUIRED)
find_package(gflags CONFIG REQUIRED)
find_package(Pangolin CONFIG REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})



add_executable(eigenMatrix SLAM_tut/src/eigenMatrix.cpp)
target_link_libraries(eigenMatrix
        Eigen3::Eigen)

add_executable(useGeometry SLAM_tut/src/useGeometry.cpp)
target_link_libraries(useGeometry
        Eigen3::Eigen)

add_executable(plotTrajectory SLAM_tut/src/plotTrajectory.cpp)
target_link_libraries(plotTrajectory
        Eigen3::Eigen
        ${Pangolin_LIBRARIES})
