#-------------------------------------------------------------------------------
#
# Try to find OpenTissue, runtime libraries, and include paths.
# Once done this will define
#
# OPENTISSUE_FOUND        : System has OpenTissue and it can be used. 
# OPENTISSUE_INCLUDE_DIRS : Directories need to find header-files used by OpenTissue and OpenTissue itself
# OPENTISSUE_LIBRARY_DIRS : Directories where the compiler can find static libraries of third-party software used by OpenTissue.
# OPENTISSUE_LIBS         : Full paths to all libraries of third-party software used by OpenTissue.
# OPENTISSUE_FLAGS        : OpenTissue compiler flags that should be used for any application using OpenTissue.
#
# Boost_unit_test_framework_LIBRARY : OpenTissue uses the Boost unit-test framework. This tells where the libs are for the linker
#

SET( OPENTISSUE_FOUND 1 )
SET(OPENTISSUE_INCLUDE_DIRS "/Users/kenny/Documents/OpenTissue_sandbox;/Users/kenny/Documents/OpenTissue_sandbox/third_party/include;/opt/local/include;/System/Library/Frameworks/OpenGL.framework;/System/Library/Frameworks/GLUT.framework/Headers;/opt/local/include")
SET(OPENTISSUE_LIBRARY_DIRS "/Users/kenny/Documents/OpenTissue_sandbox/third_party/lib/linux")
SET(OPENTISSUE_LIBS "/System/Library/Frameworks/AGL.framework;/System/Library/Frameworks/OpenGL.framework;-framework GLUT;-framework Cocoa;/opt/local/lib/libGLEW.dylib;/usr/lib/libatlas.dylib;/usr/lib/liblapack.dylib;/usr/lib/libcblas.dylib;Triangle;TetGen;TinyXML")
SET(OPENTISSUE_FLAGS "-DTIXML_USE_STL")
