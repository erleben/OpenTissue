#----------------------------------------------------------------------
#
# We got a bunch of compile tests, it really make no sense to build an
# executable from these. We just create a static lib for each of these.
#
FILE( GLOB_RECURSE FILES src/simulator_types_compile_testing/*.cpp )

FOREACH(file ${FILES})
  GET_FILENAME_COMPONENT(FILE_NAME ${file} NAME_WE )  
  ADD_LIBRARY("unit_mbd_${FILE_NAME}"    ${file}    )
  TARGET_LINK_LIBRARIES("unit_mbd_${FILE_NAME}" ${OPENTISSUE_LIBS})
ENDFOREACH( file ${FILES})

#----------------------------------------------------------------------
#
# This is the only executable in the retro unit test
#
ADD_EXECUTABLE(unit_multibody 
  src/unit_retro.cpp
  src/projected_gauss_seidel_compile_test.cpp
  src/math_policies_compile_test.cpp
  src/matrix_setup.h
  src/compile_test.cpp
)

TARGET_LINK_LIBRARIES(unit_multibody ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_multibody
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_multibody unit_multibody )
