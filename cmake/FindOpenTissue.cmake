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
# find_package(OpenTissue)
# if(OPENTISSUE_FOUND)
#  include_directories( ${OPENTISSUE_INCLUDE_DIRS} )
#  link_directories( ${OPENTISSUE_LIBRARY_DIRS} )
#  add_definitions( ${OPENTISSUE_FLAGS} )
# endif(OPENTISSUE_FOUND)
#
# And for each target using OpenTissue one would have to use
#
#  target_link_libraries(target1 ${OPENTISSUE_LIBS})
#
#


# 2020-07-25 Kenny: Cg is no longer running on osx hence, we outcommented its usage.
#-----------------------------------------------------------------------------
#
# Try to find Cg (http://developer.nvidia.com/page/cg_main.html)(as of may
# 2007, this Cg module is not part of the CMake installation, until it does we
# bundled it with OpenTissue)
#
# Observe that one only need Cg if one uses any of the OpenTissue Cg tools.
#
#INCLUDE(${OPENTISSUE_INCLUDE_DIR}/cmake/FindCg.cmake)
#mark_as_advanced(CG_COMPILER CG_INCLUDE_PATH CG_LIBRARY CG_GL_LIBRARY)
#if(FOUND_CG)
#  set(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} "${CG_INCLUDE_PATH}")
#  set( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${CG_LIBRARY} ${CG_GL_LIBRARY}  )
#endif(FOUND_CG)

#-----------------------------------------------------------------------------
#
# Boost bindings made easy, simply place them in third_party folder, then one
# wont mess up ones boost installation...
#
# BOOST_BINDINGS_PATH points to the location where boost bindings are installed.
# This makes it possible for end-users to override our default location if a
# differnet behaviour is wanted.
#
find_path(BOOST_BINDINGS_PATH  /boost/numeric/bindings/atlas/clapack.hpp
  "${OPENTISSUE_INCLUDE_DIR}/third_party/include/boost_bindings"
  DOC "What is the path where the boost bindings can be found"
  )
if(BOOST_BINDINGS_PATH)
  set(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} ${BOOST_BINDINGS_PATH})
  mark_as_advanced(BOOST_BINDINGS_PATH)
endif(BOOST_BINDINGS_PATH)

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
if(WIN32)
  FIND_LIBRARY(ATLAS_LIB atlas_local
    "${OPENTISSUE_INCLUDE_DIR}/third_party/lib/windows/"
    DOC "What is the path where the file atlas_local.lib can be found"
    )
  if(ATLAS_LIB)
    set( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${ATLAS_LIB} )
    mark_as_advanced(ATLAS_LIB)
  endif(ATLAS_LIB)
ELSE(WIN32)
  FIND_LIBRARY(ATLAS_LIB atlas
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    DOC "What is the path where the file libatlas.la can be found"
    )
  if(ATLAS_LIB)
    set( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${ATLAS_LIB} )
    mark_as_advanced(ATLAS_LIB)
  endif(ATLAS_LIB)
  FIND_LIBRARY(LAPACK_LIB lapack
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    DOC "What is the path where the file liblapack.la can be found"
    )
  if(LAPACK_LIB)
    set( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${LAPACK_LIB} )
    mark_as_advanced(LAPACK_LIB)
  endif(LAPACK_LIB)
  FIND_LIBRARY(LAPACK_ATLAS_LIB lapack_atlas
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    DOC "What is the path where the file liblapack_atlas.a can be found"
    )
  if(LAPACK_ATLAS_LIB)
    set( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${LAPACK_ATLAS_LIB} )
    mark_as_advanced(LAPACK_ATLAS_LIB)
  endif(LAPACK_ATLAS_LIB)
  FIND_LIBRARY(CBLAS_LIB cblas
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    DOC "What is the path where the file libcblas.la can be found"
    )
  if(CBLAS_LIB)
    set( OPENTISSUE_LIBS  ${OPENTISSUE_LIBS} ${CBLAS_LIB} )
    mark_as_advanced(CBLAS_LIB)
  endif(CBLAS_LIB)
  find_path(ATLAS_INCLUDE_DIR atlas/cblas.h
    /usr/include
    /usr/local/include
    /opt/local/include
    DOC "What is the path where the file atlas/cblas.h can be found"
    )
  if(ATLAS_INCLUDE_DIR)
    set(OPENTISSUE_INCLUDE_DIRS ${OPENTISSUE_INCLUDE_DIRS} ${ATLAS_INCLUDE_DIR})
    mark_as_advanced(ATLAS_INCLUDE_DIR)
  endif(ATLAS_INCLUDE_DIR)
endif(WIN32)
