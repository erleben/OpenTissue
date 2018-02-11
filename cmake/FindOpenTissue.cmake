#-------------------------------------------------------------------------------
#
# Try to find OpenTissue, runtime libraries, and include paths.
# Once done this will define
#
# OPENTISSUE_FOUND        : system has OpenTissue and it can be used. 
# OPENTISSUE_INCLUDE_DIRS : Directories need to find header-files used by OpenTissue and OpenTissue itself
# OPENTISSUE_LIBRARY_DIRS : Directories where the compiler can find static libraries of third-party software used by OpenTissue.
# OPENTISSUE_LIBS         : Full paths to all libraries of third-party software used by OpenTissue.
# OPENTISSUE_FLAGS        : OpenTissue compiler flags that should be used for any application using OpenTissue.
# 
# Typical usage is
#
# FIND_PACKAGE(OpenTissue)
# IF(OPENTISSUE_FOUND)
#  INCLUDE_DIRECTORIES( ${OPENTISSUE_INCLUDE_DIRS} )
#  LINK_DIRECTORIES( ${OPENTISSUE_LIBRARY_DIRS} )
#  ADD_DEFINITIONS( ${OPENTISSUE_FLAGS} )
# ENDIF(OPENTISSUE_FOUND)
#
# And for each target using OpenTissue one would have to use
#
#  TARGET_LINK_LIBRARIES(target1 ${OPENTISSUE_LIBS})
#
#

#-------------------------------------------------------------------------------
#
# Now we try to see if we can find OpenTissue on the local machine
#
FIND_PATH(OPENTISSUE_INCLUDE_DIR OpenTissue/configuration.h.in
  ${PROJECT_SOURCE_DIR}/
  ${PROJECT_SOURCE_DIR}/OpenTissue/
  DOC "What is the path where the file OpenTissue/configuration.h can be found"
  )

#-------------------------------------------------------------------------------
#
# Before we start to see what can be found we initialize all variables to some
#  meaningfull (ie. non-harmfull) values.
#
SET(OPENTISSUE_INCLUDE_DIRS "")
SET(OPENTISSUE_LIBRARY_DIRS "")
SET(OPENTISSUE_LIBS "")
SET(OPENTISSUE_FLAGS "")
IF (OPENTISSUE_INCLUDE_DIR)
  SET( OPENTISSUE_FOUND 1 CACHE STRING "Set to 1 if OpenTissue is found, 0 otherwise")
ELSE (OPENTISSUE_INCLUDE_DIR)
  SET( OPENTISSUE_FOUND 0 CACHE STRING "Set to 1 if OpenTissue is found, 0 otherwise")
ENDIF (OPENTISSUE_INCLUDE_DIR)

#-----------------------------------------------------------------------------
#
# Setup default include and linker directories
#
IF(WIN32)
  SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} "${OPENTISSUE_INCLUDE_DIR}")
  SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} "${OPENTISSUE_INCLUDE_DIR}/third_party/include")
  SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} "${OPENTISSUE_INCLUDE_DIR}/third_party/include/windows")
  SET(OPENTISSUE_LIBRARY_DIRS ${OPENTISSUE_LIBRARY_DIRS} "${OPENTISSUE_INCLUDE_DIR}/third_party/lib/windows")
ELSE(WIN32)
  SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} "${OPENTISSUE_INCLUDE_DIR}")
  SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} "${OPENTISSUE_INCLUDE_DIR}/third_party/include")
  SET(OPENTISSUE_LIBRARY_DIRS ${OPENTISSUE_LIBRARY_DIRS} "${OPENTISSUE_INCLUDE_DIR}/third_party/lib/linux")
ENDIF(WIN32)

#-----------------------------------------------------------------------------
#
# Try to find Cg (http://developer.nvidia.com/page/cg_main.html)(as of may
# 2007, this Cg module is not part of the CMake installation, until it does we
# bundled it with OpenTissue)
#
# Observe that one only need Cg if one uses any of the OpenTissue Cg tools.
#
INCLUDE(${OPENTISSUE_INCLUDE_DIR}/cmake/FindCg.cmake)
MARK_AS_ADVANCED(CG_COMPILER CG_INCLUDE_PATH CG_LIBRARY CG_GL_LIBRARY)
IF(FOUND_CG)
  SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} "${CG_INCLUDE_PATH}")
  SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${CG_LIBRARY} ${CG_GL_LIBRARY}  )
ENDIF(FOUND_CG)

#-----------------------------------------------------------------------------
#
# Boost bindings made easy, simply place them in third_party folder, then one
# wont mess up ones boost installation...
#
# BOOST_BINDINGS_PATH points to the location where boost bindings are installed.
# This makes it possible for end-users to override our default location if a
# differnet behaviour is wanted.
#
FIND_PATH(BOOST_BINDINGS_PATH  /boost/numeric/bindings/atlas/clapack.hpp
  "${OPENTISSUE_INCLUDE_DIR}/third_party/include/boost_bindings"
  DOC "What is the path where the boost bindings can be found"
  )
IF(BOOST_BINDINGS_PATH)
  SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} ${BOOST_BINDINGS_PATH})
  MARK_AS_ADVANCED(BOOST_BINDINGS_PATH)
ENDIF(BOOST_BINDINGS_PATH)

#-----------------------------------------------------------------------------
#
# Try to find Boost (http://www.boost.org/), Boost is needed by almost all
# OpenTissue code, one have to make sure this one works!
#
SET(Boost_USE_STATIC_LIBS ON)
#SET(Boost_USE_MULTITHREAD OFF)
FIND_PACKAGE( Boost 1.39.0 COMPONENTS unit_test_framework )
IF(Boost_FOUND)
  SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
ENDIF(Boost_FOUND)

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
FIND_PACKAGE(OpenGL)
IF(OPENGL_FOUND)
  SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR})
  SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${OPENGL_LIBRARIES}  )
ENDIF(OPENGL_FOUND)

#-----------------------------------------------------------------------------
#
# Try to find GLUT (read more here http://www.opengl.org/resources/libraries/).
#
# Glut is only needed by the Demo applications that comes with OpenTissue.
# End-users that are writting their own applications do not need this GLUT
# depedency and can ignore it.
#
IF (WIN32)
  FIND_PATH(GLUT_INCLUDE_DIR GL/glut.h
    "${OPENTISSUE_INCLUDE_DIR}/third_party/include/windows/"
    "C:/Program Files/NVIDIA Corporation/Cg/include/"
    "C:/Program Files/NVIDIA Corporation/SDK 9.5/inc/"
    "C:/Program Files/Microsoft Visual Studio 8/VC/PlatformSDK/Include/"
    DOC "What is the path where the file glut.h can be found"
    )
  IF(GLUT_INCLUDE_DIR)
    SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} ${GLUT_INCLUDE_DIR})
    MARK_AS_ADVANCED(GLUT_INCLUDE_DIR)
  ENDIF(GLUT_INCLUDE_DIR)
  FIND_LIBRARY(GLUT32_LIB glut32
    "${OpenTissue_SOURCE_DIR}/third_party/lib/windows/"
    "C:/Program Files/NVIDIA Corporation/Cg/lib"
    "C:/Program Files/Microsoft Visual Studio 8/VC/PlatformSDK/lib"
    DOC "What is the path where the file glut32.lib can be found"
    )
  IF(GLUT32_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} "optimized" ${GLUT32_LIB} )
    MARK_AS_ADVANCED(GLUT32_LIB)
  ENDIF(GLUT32_LIB)
  FIND_LIBRARY(GLUT32D_LIB glut32D
    "${OPENTISSUE_INCLUDE_DIR}/third_party/lib/windows/"
    "C:/Program Files/NVIDIA Corporation/Cg/lib"
    "C:/Program Files/Microsoft Visual Studio 8/VC/PlatformSDK/lib"
    DOC "What is the path where the file glut32.lib can be found"
    )
  IF(GLUT32D_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} "debug" ${GLUT32D_LIB} )
    MARK_AS_ADVANCED(GLUT32D_LIB)
  ENDIF(GLUT32D_LIB)
ELSE(WIN32)
  FIND_PACKAGE(GLUT)
  IF(GLUT_FOUND)
    SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} ${GLUT_INCLUDE_DIR})
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${GLUT_LIBRARIES}  )
  ENDIF(GLUT_FOUND)
ENDIF(WIN32)

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
IF(WIN32)
  FIND_LIBRARY(GLEW32_LIB glew32
    "${OPENTISSUE_INCLUDE_DIR}/third_party/lib/windows/"
    "C:/Program Files/NVIDIA Corporation/NVIDIA OpenGL SDK 10/common/GLEW/lib"
    "C:/Program Files/NVIDIA Corporation/Cg/lib"
    "C:/Program Files/Microsoft Visual Studio 8/VC/PlatformSDK/lib"
    DOC "What is the path where the file glew32.lib can be found"
    )
  IF(GLEW32_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} "optimized" ${GLEW32_LIB} )
    MARK_AS_ADVANCED(GLEW32_LIB)
  ENDIF(GLEW32_LIB)
  FIND_LIBRARY(GLEW32D_LIB glew32d
    "${OPENTISSUE_INCLUDE_DIR}/third_party/lib/windows/"
    "C:/Program Files/NVIDIA Corporation/NVIDIA OpenGL SDK 10/common/GLEW/lib"
    "C:/Program Files/NVIDIA Corporation/Cg/lib"
    "C:/Program Files/Microsoft Visual Studio 8/VC/PlatformSDK/lib"
    DOC "What is the path where the file glew32.lib can be found"
    )
  IF(GLEW32D_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} "debug" ${GLEW32D_LIB} )
    MARK_AS_ADVANCED(GLEW32D_LIB)
  ENDIF(GLEW32D_LIB)
  FIND_PATH(GLEW_INCLUDE_DIR GL/glew.h
    "${OPENTISSUE_INCLUDE_DIR}/third_party/include/windows/"
    "C:/Program Files/NVIDIA Corporation/Cg/include/"
    "C:/Program Files/NVIDIA Corporation/SDK 9.5/inc/"
    "C:/Program Files/NVIDIA Corporation/NVIDIA OpenGL SDK 10/common/GLEW/include/"
    "C:/Program Files/Microsoft Visual Studio 8/VC/PlatformSDK/Include/"
    DOC "What is the path where the file glew.h can be found"
    )
  IF(GLEW_INCLUDE_DIR)
    SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} ${GLEW_INCLUDE_DIR})
    MARK_AS_ADVANCED(GLEW_INCLUDE_DIR)
  ENDIF(GLEW_INCLUDE_DIR)
ELSE(WIN32)
  FIND_LIBRARY(GLEW32_LIB GLEW
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    DOC "What is the path where the file libGLEW.a can be found"
    )
  IF(GLEW32_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${GLEW32_LIB} )
    MARK_AS_ADVANCED(GLEW32_LIB)
  ENDIF(GLEW32_LIB)
  FIND_PATH(GLEW_INCLUDE_DIR GL/glew.h
    /usr/include
    /usr/local/include
    DOC "What is the path where the file GL/glew.h can be found"
    )
  IF(GLEW_INCLUDE_DIR)
    SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} ${GLEW_INCLUDE_DIR})
    MARK_AS_ADVANCED(GLEW_INCLUDE_DIR)
  ENDIF(GLEW_INCLUDE_DIR)
ENDIF(WIN32)

#-----------------------------------------------------------------------------
#
# Try to find atlas (http://math-atlas.sourceforge.net/). By default
# OpenTissue Boost uBLAS wrapper include header files from the boost-bindings.
# This forces end-users to install the boost-bindings.
#
# Ideally end-users should only need atlas if they use boost parts of
# OpenTissue that uses atlas routines. (As of this writing, may 2007, this is
# un-tested, but it should work as explained)
#
IF(WIN32)
  FIND_LIBRARY(ATLAS_LIB atlas_local
    "${OPENTISSUE_INCLUDE_DIR}/third_party/lib/windows/"
    DOC "What is the path where the file atlas_local.lib can be found"
    )
  IF(ATLAS_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${ATLAS_LIB} )
    MARK_AS_ADVANCED(ATLAS_LIB)
  ENDIF(ATLAS_LIB)
ELSE(WIN32)
  FIND_LIBRARY(ATLAS_LIB atlas
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    DOC "What is the path where the file libatlas.la can be found"
    )
  IF(ATLAS_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${ATLAS_LIB} )
    MARK_AS_ADVANCED(ATLAS_LIB)
  ENDIF(ATLAS_LIB)
  FIND_LIBRARY(LAPACK_LIB lapack
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    DOC "What is the path where the file liblapack.la can be found"
    )
  IF(LAPACK_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${LAPACK_LIB} )
    MARK_AS_ADVANCED(LAPACK_LIB)
  ENDIF(LAPACK_LIB)
  FIND_LIBRARY(LAPACK_ATLAS_LIB lapack_atlas
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    DOC "What is the path where the file liblapack_atlas.a can be found"
    )
  IF(LAPACK_ATLAS_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${LAPACK_ATLAS_LIB} )
    MARK_AS_ADVANCED(LAPACK_ATLAS_LIB)
  ENDIF(LAPACK_ATLAS_LIB) 
  FIND_LIBRARY(CBLAS_LIB cblas
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    DOC "What is the path where the file libcblas.la can be found"
    )
  IF(CBLAS_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${CBLAS_LIB} )
    MARK_AS_ADVANCED(CBLAS_LIB)
  ENDIF(CBLAS_LIB)
  FIND_PATH(ATLAS_INCLUDE_DIR atlas/cblas.h
    /usr/include
    /usr/local/include
    /opt/local/include
    DOC "What is the path where the file atlas/cblas.h can be found"
    )
  IF(ATLAS_INCLUDE_DIR)
    SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} ${ATLAS_INCLUDE_DIR})
    MARK_AS_ADVANCED(ATLAS_INCLUDE_DIR)
  ENDIF(ATLAS_INCLUDE_DIR)
ENDIF(WIN32)

#-----------------------------------------------------------------------------
#
# Try to find DevIL (http://openil.sourceforge.net/). The OpenTissue image
# sublibrary (see OpenTissue/image/) relies on DevIL for its io-functions.
# Only if these io-routines are used is the DevIL library required.
#
IF(WIN32)
  FIND_LIBRARY(DEVIL_LIB DevIL
    "${OPENTISSUE_INCLUDE_DIR}/third_party/lib/windows/"
    DOC "What is the path where the file DevIL.lib can be found"
    )
  IF(DEVIL_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${DEVIL_LIB} )
    MARK_AS_ADVANCED(DEVIL_LIB)
  ENDIF(DEVIL_LIB)
  FIND_LIBRARY(ILU_LIB ILU
    "${OPENTISSUE_INCLUDE_DIR}/third_party/lib/windows/"
    DOC "What is the path where the file ILU.lib can be found"
    )
  IF(ILU_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${ILU_LIB} )
    MARK_AS_ADVANCED(ILU_LIB)
  ENDIF(ILU_LIB)
  FIND_LIBRARY(ILUT_LIB ILUT
    "${OPENTISSUE_INCLUDE_DIR}/third_party/lib/windows/"
    DOC "What is the path where the file ILUT.lib can be found"
    )
  IF(ILUT_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${ILUT_LIB} )
    MARK_AS_ADVANCED(ILUT_LIB)
  ENDIF(ILUT_LIB)
  FIND_PATH(DEVIL_INCLUDE_DIR IL/il.h
    "${OPENTISSUE_INCLUDE_DIR}/third_party/include/windows/"
    DOC "What is the path where the file il.h can be found"
    )
  IF(DEVIL_INCLUDE_DIR)
    SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} ${DEVIL_INCLUDE_DIR})
    MARK_AS_ADVANCED(DEVIL_INCLUDE_DIR)
  ENDIF(DEVIL_INCLUDE_DIR)
ELSE(WIN32)
  FIND_LIBRARY(IL_LIB IL 
    /usr/lib
    /usr/local/lib
    DOC "What is the path where the file libIL.so can be found"
    )
  IF(IL_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${IL_LIB} )
    MARK_AS_ADVANCED(IL_LIB)
  ENDIF(IL_LIB)
  FIND_LIBRARY(ILU_LIB ILU 
    /usr/lib
    /usr/local/lib
    DOC "What is the path where the file libILU.so can be found"
    )
  IF(ILU_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${ILU_LIB} )
    MARK_AS_ADVANCED(ILU_LIB)
  ENDIF(ILU_LIB)
  FIND_LIBRARY(ILUT_LIB ILUT 
    /usr/lib
    /usr/local/lib
    DOC "What is the path where the file libILUT.so can be found"
    )
  IF(ILUT_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${ILUT_LIB} )
    MARK_AS_ADVANCED(ILUT_LIB)
  ENDIF(ILUT_LIB)
  FIND_PATH(DEVIL_INCLUDE_DIR IL/il.h
    /usr/include
    /usr/local/include
    DOC "What is the path where the file IL/il.h can be found"
    )
  IF(DEVIL_INCLUDE_DIR)
    SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} ${DEVIL_INCLUDE_DIR})
    MARK_AS_ADVANCED(DEVIL_INCLUDE_DIR)
  ENDIF(DEVIL_INCLUDE_DIR)
ENDIF(WIN32)

#-----------------------------------------------------------------------------
#
# Try to find Triangle (http://www.cs.cmu.edu/~quake/triangle.html). As of
# this writing (may 2007) no current sub-libraries of OpenTissue uses the
# Triangle library. However such dependencies are expected in the future.
#
IF(WIN32)
  SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} "optimized" Triangle )
  SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} "debug" TriangleD )
ELSE(WIN32)
  SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} Triangle )
ENDIF(WIN32)


#-----------------------------------------------------------------------------
#
# Try to find QHull (http://www.qhull.org/). Our mesh and t4mesh utilities
# make use of QHull for both convex hull as well as Delaunay triangulations.
#
IF(WIN32)
  SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} "optimized" qhull )
  SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} "debug" qhullD )
ELSE(WIN32)
  FIND_LIBRARY(QHULL_LIB qhull
    /usr/lib
    /usr/local/lib
    DOC "What is the path where the file libqhull.la can be found"
    )
  IF(QHULL_LIB)
    SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${QHULL_LIB} )
    MARK_AS_ADVANCED(QHULL_LIB)
  ENDIF(QHULL_LIB)
  FIND_PATH(QHULL_INCLUDE_DIR qhull/qhull.h
    /usr/include
    /usr/local/include
    DOC "What is the path where the file qhull/qhull.h can be found"
    )
  IF(QHULL_INCLUDE_DIR)
    SET(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} ${QHULL_INCLUDE_DIR})
    MARK_AS_ADVANCED(QHULL_INCLUDE_DIR)
  ENDIF(QHULL_INCLUDE_DIR)
ENDIF(WIN32)

#-----------------------------------------------------------------------------
#
# Try to find TetGen (http://tetgen.berlios.de/). OpenTissue provides a TetGen
# wrapper, it allows end-users to invoke TetGen more easily form within their
# own code.
#
IF(WIN32)
  SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} "optimized" TetGen )
  SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} "debug" TetGenD )
ELSE(WIN32)
  SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} TetGen )
ENDIF(WIN32)

#-----------------------------------------------------------------------------
#
# Try to find TinyXML (http://sourceforge.net/projects/tinyxml/). Many of
# OpenTissue i/o routines use XML format and TinyXML is our choice for parsing
# XML files. This library is used throughout OpenTissue. End-users are only
# required to have this if they use the respective i/o routines.
#
IF(WIN32)
  SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} "optimized" TinyXML )
  SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} "debug" TinyXMLD )
ELSE(WIN32)
  SET( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} TinyXML )
ENDIF(WIN32)

#-------------------------------------------------------------------------------
#
# Every demo application or unit test need to have this flag defined otherwise
# it can not use TinyXML correctly!
#
SET(OPENTISSUE_FLAGS ${OPENTISSUE_FLAGS} "-DTIXML_USE_STL")

