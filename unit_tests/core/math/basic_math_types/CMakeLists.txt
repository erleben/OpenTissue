ADD_EXECUTABLE(unit_basic_math_types src/unit_basic_math_types.cpp)

TARGET_LINK_LIBRARIES(unit_basic_math_types ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_basic_math_types 
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_basic_math_types unit_basic_math_types )