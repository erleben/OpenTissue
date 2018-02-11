ADD_EXECUTABLE(unit_get_environment_variable src/unit_get_environment_variable.cpp)

TARGET_LINK_LIBRARIES(unit_get_environment_variable ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_get_environment_variable
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_get_environment_variable unit_get_environment_variable )
