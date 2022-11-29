cmake_minimum_required(VERSION 3.16)
project(SLAM_tut)

set(CMAKE_CXX_STANDARD 17)

if (WIN32)
    find_package(Eigen3 CONFIG REQUIRED)
    find_package(gflags CONFIG REQUIRED)
    find_package(Pangolin CONFIG REQUIRED)
    find_package(Sophus CONFIG REQUIRED)
else()
    find_package(Eigen3 REQUIRED)
    find_package(gflags REQUIRED)
    find_package(Pangolin REQUIRED)
    find_package(Sophus REQUIRED)
endif()


add_executable(eigenMatrix SLAM_tut/src/eigenMatrix.cpp)
target_link_libraries(eigenMatrix
        Eigen3::Eigen)

add_executable(useGeometry SLAM_tut/src/useGeometry.cpp)
target_link_libraries(useGeometry
        Eigen3::Eigen)

add_executable(plotTrajectory SLAM_tut/src/plotTrajectory.cpp)
target_include_directories(plotTrajectory PRIVATE ${Pangolin_INCLUDE_DIRS})
target_link_libraries(plotTrajectory
        Eigen3::Eigen
        ${Pangolin_LIBRARIES})

add_executable(useSophus SLAM_tut/src/useSophus.cpp)
target_link_libraries(useSophus
        Eigen3::Eigen
        Sophus::Sophus)

add_executable(trajectoryError SLAM_tut/src/trajectoryError.cpp)
target_include_directories(trajectoryError PRIVATE ${Pangolin_INCLUDE_DIRS})
target_link_libraries(trajectoryError
        Eigen3::Eigen
        ${Pangolin_LIBRARIES}
        Sophus::Sophus)
