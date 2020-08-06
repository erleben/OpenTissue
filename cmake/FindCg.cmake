#
# Try to find nVidia's Cg compiler, runtime libraries, and include path.
# Once done this will define
#
# CG_FOUND        - system has NVidia Cg and it can be used.
# CG_INCLUDE_PATH = directory where cg.h resides
# CG_LIBRARY = full path to libCg.so (Cg.DLL on win32)
# CG_GL_LIBRARY = full path to libCgGL.so (CgGL.dll on win32)
# CG_COMPILER = full path to cgc (cgc.exe on win32)
#

# On OSX default to using the framework version of Cg.

if (APPLE)
  INCLUDE(${CMAKE_ROOT}/Modules/CMakeFindFrameworks.cmake)
  set(CG_FRAMEWORK_INCLUDES)
  CMAKE_FIND_FRAMEWORKS(Cg)
  if (Cg_FRAMEWORKS)
    foreach(dir ${Cg_FRAMEWORKS})
      set(CG_FRAMEWORK_INCLUDES ${CG_FRAMEWORK_INCLUDES}
        ${dir}/Headers ${dir}/PrivateHeaders)
    endforeach(dir)

    #Find the include  dir
    find_path(CG_INCLUDE_PATH cg.h
      ${CG_FRAMEWORK_INCLUDES}
      )

    #Since we are using Cg framework, we must link to it.
    set(CG_LIBRARY "-framework Cg" CACHE STRING "Cg library")
    set(CG_GL_LIBRARY "-framework Cg" CACHE STRING "Cg GL library")
  endif (Cg_FRAMEWORKS)
  FIND_PROGRAM(CG_COMPILER cgc
    /usr/bin
    /usr/local/bin
    DOC "The Cg compiler"
    )
ELSE (APPLE)
  if (WIN32)
    FIND_PROGRAM( CG_COMPILER cgc
	"D:/Program Files/NVIDIACorporation/Cg/bin"
      "C:/Program Files/NVIDIA Corporation/Cg/bin"
      "C:/Program Files/Cg"
      ${PROJECT_SOURCE_DIR}/../Cg
      DOC "The Cg Compiler"
      )
    if (CG_COMPILER)
      get_filename_component(CG_COMPILER_DIR ${CG_COMPILER} PATH)
      get_filename_component(CG_COMPILER_SUPER_DIR ${CG_COMPILER_DIR} PATH)
    ELSE (CG_COMPILER)
      set (CG_COMPILER_DIR .)
      set (CG_COMPILER_SUPER_DIR ..)
    endif (CG_COMPILER)
    find_path( CG_INCLUDE_PATH Cg/cg.h
	"D:/Program Files/NVIDIACorporation/Cg/include"
      "C:/Program Files/NVIDIA Corporation/Cg/include"
      "C:/Program Files/Cg"
      ${PROJECT_SOURCE_DIR}/../Cg
      ${CG_COMPILER_SUPER_DIR}/include
      ${CG_COMPILER_DIR}
      DOC "The directory where Cg/cg.h resides"
      )
    FIND_LIBRARY( CG_LIBRARY
      NAMES Cg
      PATHS
     	"D:/Program Files/NVIDIACorporation/Cg/lib"
	"C:/Program Files/NVIDIA Corporation/Cg/lib"
      "C:/Program Files/Cg"
      ${PROJECT_SOURCE_DIR}/../Cg
      ${CG_COMPILER_SUPER_DIR}/lib
      ${CG_COMPILER_DIR}
      DOC "The Cg runtime library"
      )
    FIND_LIBRARY( CG_GL_LIBRARY
      NAMES CgGL
      PATHS
   	"D:/Program Files/NVIDIACorporation/Cg/lib"
      "C:/Program Files/NVIDIA Corporation/Cg/lib"
      "C:/Program Files/Cg"
      ${PROJECT_SOURCE_DIR}/../Cg
      ${CG_COMPILER_SUPER_DIR}/lib
      ${CG_COMPILER_DIR}
      DOC "The Cg runtime library"
      )
  ELSE (WIN32)
    FIND_PROGRAM( CG_COMPILER cgc
      /usr/bin
      /usr/local/bin
      DOC "The Cg Compiler"
      )
    get_filename_component(CG_COMPILER_DIR "${CG_COMPILER}" PATH)
    get_filename_component(CG_COMPILER_SUPER_DIR "${CG_COMPILER_DIR}" PATH)
    find_path( CG_INCLUDE_PATH Cg/cg.h
      /usr/include
      /usr/local/include
      ${CG_COMPILER_SUPER_DIR}/include
      DOC "The directory where Cg/cg.h resides"
      )
    FIND_LIBRARY( CG_LIBRARY Cg
      PATHS
      /usr/lib64
      /usr/lib
      /usr/local/lib64
      /usr/local/lib
      ${CG_COMPILER_SUPER_DIR}/lib64
      ${CG_COMPILER_SUPER_DIR}/lib
      DOC "The Cg runtime library"
      )
    FIND_LIBRARY( CG_GL_LIBRARY CgGL
      PATHS
      /usr/lib64
      /usr/lib
      /usr/local/lib64
      /usr/local/lib
      ${CG_COMPILER_SUPER_DIR}/lib64
      ${CG_COMPILER_SUPER_DIR}/lib
      DOC "The Cg runtime library"
      )
  endif (WIN32)
endif (APPLE)

if (CG_INCLUDE_PATH)
  set( FOUND_CG 1 CACHE STRING "Set to 1 if CG is found, 0 otherwise")
ELSE (CG_INCLUDE_PATH)
  set( FOUND_CG 0 CACHE STRING "Set to 1 if CG is found, 0 otherwise")
endif (CG_INCLUDE_PATH)

mark_as_advanced( FOUND_CG )
