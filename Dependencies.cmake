# Copyright (c) <year> <author> (<email>)
# Distributed under the MIT License.
# See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT

# Include script to build external libraries with CMake.
include(ExternalProject)

# ------------------------------

#add_subdirectory(external/freeglut)
#set(FREEGLUT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/freeglut/include CACHE PATH "freeglut include directory")
#set(CMAKE_PREFIX_PATH "C:/freeglut")
#set(FREEGLUT_INCLUDE_DIRS freeglut/include) 
#set(FREEGLUT_LIBRARY_DIRS freeglut/lib) 
#set(FREEGLUT_LIBRARIES freeglut) 
#include_directories(${GLUT_INCLUDE_DIRS})
#link_directories(${GLUT_LIBRARY_DIRS})
#add_definitions(${GLUT_DEFINITIONS})
#find_package(GLUT REQUIRED)
#find_package(OPENGL REQUIRED)
# if(NOT GLUT_FOUND)
#	message("Unable to find freeglut, cloning...")
#	set(FREEGLUT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} /external/freeglut  CACHE PATH "freeglut include directory")
# endif()

# Freeglut

if(BUILD_MAIN)
  if(NOT BUILD_DEPENDENCIES)
    find_package(GLUT)
  endif(NOT BUILD_DEPENDENCIES)

  if(NOT GLUT_FOUND)
    message(STATUS "Freeglut will be downloaded when ${CMAKE_PROJECT_NAME} is built")
    ExternalProject_Add(freeglut-lib
      PREFIX ${EXTERNAL_PATH}/freeglut
      #--Download step--------------
      URL https://github.com/LuaDist/freeglut/archive/master.zip
      TIMEOUT 30
      #--Update/Patch step----------
      UPDATE_COMMAND ""
      PATCH_COMMAND ""
      #--Configure step-------------
      CONFIGURE_COMMAND ""
      #--Build step-----------------
      BUILD_COMMAND ""
      #--Install step---------------
      INSTALL_COMMAND ""
      #--Output logging-------------
      LOG_DOWNLOAD ON
    )
    ExternalProject_Get_Property(freeglut-lib source_dir)
    set(FREEGLUT_INCLUDE_DIRS ${source_dir}/include CACHE INTERNAL "Path to include folder for Freeglut")
  endif(NOT GLUT_FOUND)

  if(NOT APPLE)
    include_directories(SYSTEM AFTER "${FREEGLUT_INCLUDE_DIRS}")
  else(APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem \"${FREEGLUT_INCLUDE_DIRS}\"")
  endif(NOT APPLE)
endif(BUILD_MAIN)

# -------------------------------

# Catch: https://github.com/philsquared/Catch

if(BUILD_TESTS)
  if(NOT BUILD_DEPENDENCIES)
    find_package(CATCH)
  endif(NOT BUILD_DEPENDENCIES)

  if(NOT CATCH_FOUND)
    message(STATUS "Catch will be downloaded when ${CMAKE_PROJECT_NAME} is built")
    ExternalProject_Add(catch-lib
      PREFIX ${EXTERNAL_PATH}/Catch
      #--Download step--------------
      URL https://github.com/philsquared/Catch/archive/master.zip
      TIMEOUT 30
      #--Update/Patch step----------
      UPDATE_COMMAND ""
      PATCH_COMMAND ""
      #--Configure step-------------
      CONFIGURE_COMMAND ""
      #--Build step-----------------
      BUILD_COMMAND ""
      #--Install step---------------
      INSTALL_COMMAND ""
      #--Output logging-------------
      LOG_DOWNLOAD ON
    )
    ExternalProject_Get_Property(catch-lib source_dir)
    set(CATCH_INCLUDE_DIRS ${source_dir}/include CACHE INTERNAL "Path to include folder for Catch")
  endif(NOT CATCH_FOUND)

  if(NOT APPLE)
    include_directories(SYSTEM AFTER "${CATCH_INCLUDE_DIRS}")
  else(APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem \"${CATCH_INCLUDE_DIRS}\"")
  endif(NOT APPLE)
endif(BUILD_TESTS)

# -------------------------------
