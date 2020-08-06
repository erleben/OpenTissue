cmake_minimum_required(VERSION 3.12)
project(OpenTissue VERSION 0.994
                   DESCRIPTION "A header only c++ library collection of generic algorithms and data structures for rapid development of interactive modeling and simulation."
                   HOMEPAGE_URL "https://github.com/erleben/OpenTissue")

# Add connan dependencies utilities, only access to root directories is used.
if(EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
  include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
  set(BOOST_ROOT ${CONAN_BOOST_ROOT})
  set(DevIL_ROOT ${CONAN_DEVIL_ROOT})
  set(Qhull_ROOT ${CONAN_QHULL_ROOT})
  set(TetGen_ROOT ${CONAN_TETGEN_ROOT})
  set(TinyXML_ROOT ${CONAN_TINYXML_ROOT})
  set(Triangle_ROOT ${CONAN_TRIANGLE_ROOT})
  conan_basic_setup()
endif()

#-----------------------------------------------------------------------------
#
# Avoid in-source builds.
#
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
  message(
    FATAL_ERROR
    "CMake generation for OpenTissue is not allowed within the source directory!"
    )
endif()

list(INSERT CMAKE_MODULE_PATH 0 ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

#-----------------------------------------------------------------------------
#
# Global variables that control the behaviour of CMake. Use these in
# the GUI to turn on/off different kind of things.
#
set(OPENTISSUE_ENABLE_UNIT_TESTS         FALSE   CACHE BOOL   "Build unit test")
set(OPENTISSUE_ENABLE_DOCUMENTATION      FALSE   CACHE BOOL   "Build documentation")
set(OPENTISSUE_ENABLE_DEMOS              FALSE   CACHE BOOL   "Build demos")

#-----------------------------------------------------------------------------
#
# Try to find Boost (http://www.boost.org/), Boost is needed by almost all
# OpenTissue code, one have to make sure this one works!
#
if(OPENTISSUE_ENABLE_UNIT_TESTS)
  set(OPENTISSUE_BOOST_COMPONENTS unit_test_framework)
endif()
find_package(Boost 1.39.0 COMPONENTS "${OPENTISSUE_BOOST_COMPONENTS}" REQUIRED)

if(OPENTISSUE_ENABLE_DEMOS)
  #-----------------------------------------------------------------------------
  #
  # Try to find OpenGL (http://www.opengl.org/)
  #
  # OpenGL is used by the default demo applications in OpenTissue. The
  # visualization part of OpenTissue has been designed as separate code
  # pieces. These code pieces are intended for illustration purpose and
  # for brute-force debug utilities for the OpenTissue developers. The
  # openGL code pieces have never been intended to be used by end-users.
  # Therefore end-users should be able to use OpenTissue without OpenGL.
  #
  # The only expcetion is our GPGPU code pieces, these are hard-wired to OpenGL
  # and Cg.
  #
  #
  set(OpenGL_GL_PREFERENCE GLVND)
  find_package(OpenGL REQUIRED)

  #-----------------------------------------------------------------------------
  #
  # Try to find GLUT (read more here http://www.opengl.org/resources/libraries/).
  #
  # Glut is only needed by the Demo applications that comes with OpenTissue.
  # End-users that are writting their own applications do not need this GLUT
  # depedency and can ignore it.
  #
  find_package(GLUT REQUIRED)

  #-----------------------------------------------------------------------------
  #
  # Try to get GLEW (http://glew.sourceforge.net/). Our demo application
  # framework relies on GLEW for initialization of any OpenGL extentions.
  # End-users are not required to use GLEW.
  #
  # As long as if they use any GPGPU stuff from OpenTissue they provide their
  # own extention initialization. If no GPGPU stuff is used then this library
  # dependency can be ignored.
  #
  find_package(GLEW REQUIRED)

  find_package(TinyXML REQUIRED)
endif()

#-----------------------------------------------------------------------------
#
# Try to find DevIL (http://openil.sourceforge.net/). The OpenTissue image
# sublibrary (see OpenTissue/image/) relies on DevIL for its io-functions.
# Only if these io-routines are used is the DevIL library required.
#
find_package(DevIL REQUIRED)

find_package(TetGen REQUIRED)
find_package(Triangle REQUIRED)


#-----------------------------------------------------------------------------
#
# Try to find QHull (http://www.qhull.org/). Our mesh and t4mesh utilities
# make use of QHull for both convex hull as well as Delaunay triangulations.
#
find_package(Qhull REQUIRED)

#-----------------------------------------------------------------------------
#
# Look into subfolders
#
add_subdirectory(OpenTissue)

if(OPENTISSUE_ENABLE_DEMOS)
  add_subdirectory(demos)
endif()

if(OPENTISSUE_ENABLE_DOCUMENTATION)
  add_subdirectory(documentation)
endif()

if(OPENTISSUE_ENABLE_UNIT_TESTS)
  include(CTest)
  add_subdirectory(unit_tests)
endif()

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
include(CMakePackageConfigHelpers)
include(CPack)
include(GNUInstallDirs)

export(PACKAGE OT) # Makes OT discoverable, via ~/.cmake
export(TARGETS OpenTissue NAMESPACE OT:: FILE OpenTissueTargets.cmake)

configure_file(
  "${PROJECT_SOURCE_DIR}/cmake/OpenTissueConfig.cmake.in"
  "${PROJECT_SOURCE_DIR}/OpenTissueConfig.cmake"
  @ONLY
)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/OpenTissueConfig.cmake.in"
  "${PROJECT_BINARY_DIR}/OpenTissueConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OpenTissue
  PATH_VARS CMAKE_INSTALL_LIBDIR CMAKE_INSTALL_INCLUDEDIR
)

write_basic_package_version_file(
  OpenTissueConfigVersion.cmake
  VERSION ${OpenTissue_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(TARGETS OpenTissue
  EXPORT OpenTissueTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(EXPORT OpenTissueTargets
  FILE OpenTissueTargets.cmake
  NAMESPACE OT::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OpenTissue
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/OpenTissueConfigVersion.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/OpenTissueConfig.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OpenTissue
)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/OpenTissue
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  PATTERN "*.h.in" EXCLUDE
  PATTERN "*.cmake" EXCLUDE
  PATTERN "CMakeLists.txt" EXCLUDE
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/OpenTissue/configuration.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/OpenTissue
)
