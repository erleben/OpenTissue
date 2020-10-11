#-----------------------------------------------------------------------------
# If on linux and found a dynamic link library (.so),
# define BOOST_TEST_DYN_LINK to automagically get the main method.
# TODO: This needs to be fixed, Boost_unit_test_framework_LIBRARY is not always defined.
if(Boost_unit_test_framework_LIBRARY)
  get_filename_component(LIB_EXTENSION ${Boost_unit_test_framework_LIBRARY} EXT)
  if(LIB_EXTENSION STREQUAL ".so")
    add_definitions(-DBOOST_TEST_DYN_LINK)
  endif(LIB_EXTENSION STREQUAL ".so")
endif(Boost_unit_test_framework_LIBRARY)

function(ot_add_test TEST_NAME)
  add_test(NAME ${TEST_NAME}
    COMMAND ${CMAKE_COMMAND} -E env OPENTISSUE=${PROJECT_SOURCE_DIR} $<TARGET_FILE:${TEST_NAME}> ${ARGN}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )
endfunction()

option(OPENTISSUE_ENABLE_CODE_COVERAGE "Enable coverage reporting" OFF)
if(OPENTISSUE_ENABLE_CODE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # Add required flags (GCC & LLVM/Clang)
  target_compile_options(OpenTissue INTERFACE
    -O0        # no optimization
    -g         # generate debug info
    --coverage # sets all required flags
  )
  if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.13)
    target_link_options(OpenTissue INTERFACE --coverage)
  else()
    target_link_libraries(OpenTissue INTERFACE --coverage)
  endif()
endif()

add_subdirectory( collision )
add_subdirectory( core )
add_subdirectory( dynamics )
add_subdirectory( kinematics )
add_subdirectory( utility )
add_subdirectory( io )
