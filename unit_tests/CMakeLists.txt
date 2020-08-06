#-----------------------------------------------------------------------------
# If on linux and found a dynamic link library (.so),
# define BOOST_TEST_DYN_LINK to automagically get the main method.
#
if(Boost_unit_test_framework_LIBRARY)
  get_filename_component(LIB_EXTENSION ${Boost_unit_test_framework_LIBRARY} EXT)
  if(LIB_EXTENSION STREQUAL ".so")
    add_definitions(-DBOOST_TEST_DYN_LINK)
  endif(LIB_EXTENSION STREQUAL ".so")
endif(Boost_unit_test_framework_LIBRARY)

function(ot_add_test TEST_NAME)
  add_test(NAME ${TEST_NAME}
    COMMAND $<TARGET_FILE:${TEST_NAME}> ${ARGN}
  )
endfunction()

add_subdirectory( collision )
add_subdirectory( core )
add_subdirectory( dynamics )
add_subdirectory( kinematics )
add_subdirectory( utility )
