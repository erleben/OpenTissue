#-----------------------------------------------------------------------------
# If on linux and found a dynamic link library (.so), 
# define BOOST_TEST_DYN_LINK to automagically get the main method.
#
IF(Boost_unit_test_framework_LIBRARY)
  GET_FILENAME_COMPONENT(LIB_EXTENSION ${Boost_unit_test_framework_LIBRARY} EXT)
  IF(LIB_EXTENSION STREQUAL ".so")
    ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)
  ENDIF(LIB_EXTENSION STREQUAL ".so")
ENDIF(Boost_unit_test_framework_LIBRARY)

SUBDIRS( collision )
SUBDIRS( core )
SUBDIRS( dynamics )
SUBDIRS( kinematics )
SUBDIRS( utility )
