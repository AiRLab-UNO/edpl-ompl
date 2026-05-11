cmake_minimum_required(VERSION 3.20)

if(${CMAKE_VERSION} VERSION_GREATER 2.8.3)
    cmake_policy(SET CMP0017 NEW)
endif()
if(NOT ${CMAKE_VERSION} VERSION_LESS 3.0.0)
    cmake_policy(SET CMP0042 NEW)
endif()

project(bspapp CXX C)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Type of build" FORCE)
endif()

message(STATUS "Building ${CMAKE_BUILD_TYPE}")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
    "/usr/share/cmake-2.8/Modules/"
    "${PROJECT_SOURCE_DIR}/CMakeModules/")

# Required packages
find_package(Boost REQUIRED COMPONENTS
    date_time thread serialization filesystem system
    program_options unit_test_framework chrono)
find_package(OMPL REQUIRED)
find_package(Armadillo REQUIRED)
# SQLite3
find_package(PkgConfig REQUIRED)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)
find_package(YAML-CPP REQUIRED)

# FCL (and its transitive dependency libccd)
find_library(FCL_LIBRARY fcl REQUIRED)
find_path(FCL_INCLUDE_DIR fcl/fcl.h)
find_library(CCD_LIBRARY ccd REQUIRED)

# assimp
find_library(ASSIMP_LIBRARY assimp REQUIRED)
find_path(ASSIMP_INCLUDE_DIR assimp/Importer.hpp)

include_directories(
    ${PROJECT_SOURCE_DIR}/include/
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
    /usr/include/eigen3
    ${YAML_CPP_INCLUDE_DIR}
    ${SQLITE3_INCLUDE_DIRS}
    ${FCL_INCLUDE_DIR}
    ${ASSIMP_INCLUDE_DIR}
)

link_directories(
    ${Boost_LIBRARY_DIRS}
    ${OMPL_LIBRARY_DIRS}
    ${SQLITE3_LIBRARY_DIRS}
)

add_library(bsp_lib
    src/MotionModels/TwoDPointMotionModel.cpp
    src/MotionModels/OmnidirectionalMotionModel.cpp
    src/MotionModels/UnicycleMotionModel.cpp
    src/MotionModels/UnicycleStatePropagator.cpp
    src/MotionModels/OmnidirectionalStatePropagator.cpp
    src/MotionModels/TwoDPointStatePropagator.cpp
    src/ObservationModels/CamAruco2DObservationModel.cpp
    src/ObservationModels/TwoDBeaconObservationModel.cpp
    src/ObservationModels/HeadingBeaconObservationModel.cpp
    src/Planner/FIRM.cpp
    src/Planner/FIRMCP.cpp
    src/Planner/NBM3P.cpp
    src/Samplers/GaussianValidBeliefSampler.cpp
    src/Samplers/UniformValidBeliefSampler.cpp
    src/SeparatedControllers/RHCICreate.cpp
    src/SeparatedControllers/FiniteTimeLQR.cpp
    src/SeparatedControllers/StationaryLQR.cpp
    src/SpaceInformation/SpaceInformation.cpp
    src/Spaces/SE2BeliefSpace.cpp
    src/Spaces/R2BeliefSpace.cpp
    src/Utils/FIRMUtils.cpp
    src/Visualization/Visualizer.cpp
    src/Visualization/Window.cpp
    src/Visualization/GLWidget.cpp
    src/Filters/ExtendedKF.cpp
    src/Filters/LinearizedKF.cpp
)

target_link_libraries(bsp_lib
    ${Boost_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    ${OMPL_LIBRARY}
    ${SQLITE3_LIBRARIES}
    ${FCL_LIBRARY}
    ${ASSIMP_LIBRARY}
)

add_executable(bsp-app-demo src/main_firm.cpp)
target_link_libraries(bsp-app-demo
    bsp_lib
    ${FCL_LIBRARY}
    ${CCD_LIBRARY}
)

add_executable(firmcp-demo src/main_firmcp.cpp)
target_link_libraries(firmcp-demo
    bsp_lib
    ${FCL_LIBRARY}
    ${CCD_LIBRARY}
)
