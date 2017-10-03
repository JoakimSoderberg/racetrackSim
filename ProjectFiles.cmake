# Copyright (c) <year> <author> (<email>)
# Distributed under the MIT License.
# See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT

# Set project source files.
set(SRC
  "${SRC_PATH}/GlutVizualisation.cpp"
  "${SRC_PATH}/bitmap.c"
  "${SRC_PATH}/Geometry.cpp"
  "${SRC_PATH}/RRTPlanner.cpp"
  "${SRC_PATH}/DrawFunc.cpp"
  "${SRC_PATH}/MissionHandler.cpp"
  "${SRC_PATH}/MapHandler.cpp"
  "${SRC_PATH}/Simulator.cpp"
  "${SRC_PATH}/OptimalTrajectory.cpp"
  "${SRC_PATH}/Model.cpp"
  "${SRC_PATH}/SamplingStrategies.cpp"
  "${SRC_PATH}/ObstacleMap.cpp"
  "${SRC_PATH}/ModelParameters.cpp"
  "${SRC_PATH}/PlannerTypes.cpp"
  "${SRC_PATH}/PurePursuitController.cpp"
)

# Set project main file.
set(MAIN_SRC
  "${SRC_PATH}/main.cpp"
)

# Set project test source files.
set(TEST_SRC
  "${TEST_SRC_PATH}/testCppbase.cpp"
  "${TEST_SRC_PATH}/testFactorial.cpp"
)
