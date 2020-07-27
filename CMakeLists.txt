#-----------------------------------------------------------------------------
#
# We don't allow in-source builds. 
#
IF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  MESSAGE(
    FATAL_ERROR 
    "CMake generation for OpenTissue is not allowed within the source directory!"
    )
ENDIF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
 endif(COMMAND cmake_policy)

PROJECT(OpenTissue)

SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

IF (MSVC)
  ADD_DEFINITIONS(-D_SCL_SECURE_NO_WARNINGS)
  ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)
ENDIF(MSVC)

ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
ADD_DEFINITIONS(-DGL_SILENCE_DEPRECATION)

SET(
  CMAKE_MODULE_PATH 
  ${PROJECT_SOURCE_DIR}/CMake 
  ${PROJECT_SOURCE_DIR}/cmake 
  ${CMAKE_MODULE_PATH}
  )


#-----------------------------------------------------------------------------
#
# Global variables that control the behaviour of CMake. Use these in
# the GUI to turn on/off different kind of things.
#
SET(ENABLE_UNIT_TESTS         FALSE   CACHE BOOL   "Should we add unit tests to the makefiles")
SET(ENABLE_DOCUMENTATION      TRUE    CACHE BOOL   "Should we add documentation to the makefiles")
SET(ENABLE_DEMOS              TRUE    CACHE BOOL   "Should we add documentation to the makefiles")

#-------------------------------------------------------------------------------
#
# Find all OpenTissue third-party dependencies, include paths and so on.
#
FIND_PACKAGE(OpenTissue)
IF(OPENTISSUE_FOUND)
  INCLUDE_DIRECTORIES( ${OPENTISSUE_INCLUDE_DIRS} )
  LINK_DIRECTORIES( ${OPENTISSUE_LIBRARY_DIRS} )
  ADD_DEFINITIONS( ${OPENTISSUE_FLAGS} )
ENDIF(OPENTISSUE_FOUND)

#-------------------------------------------------------------------------------
#
# Add support for other CMake applications, so it is easier to use OpenTissue.
#
# Recall, that CMake works with tree terms: source-, build- (ie. binary-) and
# isntall- configuration/tree.
#
# OpenTissue is a header only library, so the build-tree only makes sense for
# demo applications and units. Source- and install trees are basically just all
# the header-files and third-party libraries.
#
# As of this writting (May 2007) we only support FindPackage(OpenTissue)
# ``source-tree'' support. Thus if one decides to install OpenTissue using CMake
# then one would not have an OpenTissueConfig.cmake in the install-tree.
#
CONFIGURE_FILE(
  "${PROJECT_SOURCE_DIR}/cmake/OpenTissueConfig.cmake.in"
  "${PROJECT_SOURCE_DIR}/OpenTissueConfig.cmake"
  @ONLY
)

#-------------------------------------------------------------------------------
#
# Generate configuration.h file
# This way platform and system specific settings can be added to the
# configuration.h file
#
CONFIGURE_FILE(
  "${PROJECT_SOURCE_DIR}/OpenTissue/configuration.h.in"
  "${PROJECT_SOURCE_DIR}/OpenTissue/configuration.h"
  @ONLY
)

#-----------------------------------------------------------------------------
#
# Look into subfolders
#
SUBDIRS(OpenTissue)
SUBDIRS(third_party)

IF (ENABLE_DEMOS)
  SUBDIRS(demos)
ENDIF(ENABLE_DEMOS)

IF (ENABLE_DOCUMENTATION)
  SUBDIRS(documentation)
ENDIF(ENABLE_DOCUMENTATION)

IF (ENABLE_UNIT_TESTS)
  SUBDIRS(unit_tests)
ENDIF(ENABLE_UNIT_TESTS)
