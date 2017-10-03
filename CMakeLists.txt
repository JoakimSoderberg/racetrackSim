cmake_minimum_required(VERSION 2.8.12)

project(racetracksim)

set(SRCS
    "${PROJECT_SOURCE_DIR}/src/main.cpp"
    "${PROJECT_SOURCE_DIR}/src/BTHandler.cpp"
    "${PROJECT_SOURCE_DIR}/src/ClientGame_RRT.cpp"
    "${PROJECT_SOURCE_DIR}/src/ClientNetwork_RRT.cpp"
    "${PROJECT_SOURCE_DIR}/src/DistanceToCenter.cpp"
    "${PROJECT_SOURCE_DIR}/src/DrawFunc.cpp"
    "${PROJECT_SOURCE_DIR}/src/factorial.cpp"
    "${PROJECT_SOURCE_DIR}/src/Geometry.cpp"
    "${PROJECT_SOURCE_DIR}/src/GlutVizualisation.cpp"
    "${PROJECT_SOURCE_DIR}/src/LCMHandler.cpp"
    "${PROJECT_SOURCE_DIR}/src/LQ_Controller.cpp"
    "${PROJECT_SOURCE_DIR}/src/main.cpp"
    "${PROJECT_SOURCE_DIR}/src/MapHandler.cpp"
    "${PROJECT_SOURCE_DIR}/src/matrix.cpp"
    "${PROJECT_SOURCE_DIR}/src/MissionHandler.cpp"
    "${PROJECT_SOURCE_DIR}/src/Model.cpp"
    "${PROJECT_SOURCE_DIR}/src/model_truck.cpp"
    "${PROJECT_SOURCE_DIR}/src/Model2axis.cpp"
    "${PROJECT_SOURCE_DIR}/src/Model4axis.cpp"
    "${PROJECT_SOURCE_DIR}/src/ModelParameters.cpp"
    "${PROJECT_SOURCE_DIR}/src/ModelTrailer.cpp"
    "${PROJECT_SOURCE_DIR}/src/NetworkServices_RRT.cpp"
    "${PROJECT_SOURCE_DIR}/src/ObstacleMap.cpp"
    "${PROJECT_SOURCE_DIR}/src/OptimalTrajectory.cpp"
    "${PROJECT_SOURCE_DIR}/src/PlannerTypes.cpp"
    "${PROJECT_SOURCE_DIR}/src/PurePursuitController.cpp"
    "${PROJECT_SOURCE_DIR}/src/ReactivePlanner.cpp"
    "${PROJECT_SOURCE_DIR}/src/refinterface.cpp"
    "${PROJECT_SOURCE_DIR}/src/RRTPlanner.cpp"
    "${PROJECT_SOURCE_DIR}/src/RTError.cpp"
    "${PROJECT_SOURCE_DIR}/src/SamplingStrategies.cpp"
    "${PROJECT_SOURCE_DIR}/src/Simulator.cpp"
    "${PROJECT_SOURCE_DIR}/src/State.cpp"
    "${PROJECT_SOURCE_DIR}/src/TreeStructure.cpp"
    "${PROJECT_SOURCE_DIR}/src/bitmap.c")

# Only used by generators like Visual Studio that won't list the header files in the project otherwise.
set(HDRS
    "${PROJECT_SOURCE_DIR}/include/bitmap.h"
    "${PROJECT_SOURCE_DIR}/include/BTHandler.h"
    "${PROJECT_SOURCE_DIR}/include/ClientGame_RRT.h"
    "${PROJECT_SOURCE_DIR}/include/ClientNetwork_RRT.h"
    "${PROJECT_SOURCE_DIR}/include/DrawFunc.h"
    "${PROJECT_SOURCE_DIR}/include/Geometry.h"
    "${PROJECT_SOURCE_DIR}/include/globaldefines.h"
    "${PROJECT_SOURCE_DIR}/include/GlutVizualisation.h"
    "${PROJECT_SOURCE_DIR}/include/LCMHandler.h"
    "${PROJECT_SOURCE_DIR}/include/LQ_Controller.h"
    "${PROJECT_SOURCE_DIR}/include/MapHandler.h"
    "${PROJECT_SOURCE_DIR}/include/matrix.h"
    "${PROJECT_SOURCE_DIR}/include/MissionHandler.h"
    "${PROJECT_SOURCE_DIR}/include/Model.h"
    "${PROJECT_SOURCE_DIR}/include/Model2axis.h"
    "${PROJECT_SOURCE_DIR}/include/Model4axis.h"
    "${PROJECT_SOURCE_DIR}/include/ModelParameters.h"
    "${PROJECT_SOURCE_DIR}/include/ModelTrailer.h"
    "${PROJECT_SOURCE_DIR}/include/NetworkData.h"
    "${PROJECT_SOURCE_DIR}/include/NetworkServices_RRT.h"
    "${PROJECT_SOURCE_DIR}/include/ObstacleMap.h"
    "${PROJECT_SOURCE_DIR}/include/OptimalTrajectory.h"
    "${PROJECT_SOURCE_DIR}/include/Planner.h"
    "${PROJECT_SOURCE_DIR}/include/PlannerTypes.h"
    "${PROJECT_SOURCE_DIR}/include/PurePursuitController.h"
    "${PROJECT_SOURCE_DIR}/include/ReactivePlanner.h"
    "${PROJECT_SOURCE_DIR}/include/refinterface.h"
    "${PROJECT_SOURCE_DIR}/include/RRTPlanner.h"
    "${PROJECT_SOURCE_DIR}/include/RTError.h"
    "${PROJECT_SOURCE_DIR}/include/SamplingStrategies.h"
    "${PROJECT_SOURCE_DIR}/include/Simulator.h"
    "${PROJECT_SOURCE_DIR}/include/State.h"
    "${PROJECT_SOURCE_DIR}/include/TreeStructure.h")

add_executable(racetracksim ${SRCS} ${HDRS})


# TODO: "proper way"
#find_package(GLUT REQUIRED)

# For now we hard code it
set(GLUT_LIBRARIES "" CACHE STRING "Path to glut library")
set(GLUT_INCLUDE_DIR "" CACHE STRING "Path to glut include dir")

target_link_libraries(racetracksim ${GLUT_LIBRARIES})
include_directories(${GLUT_INCLUDE_DIR})
