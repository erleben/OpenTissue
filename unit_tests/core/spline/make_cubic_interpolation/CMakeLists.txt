ADD_EXECUTABLE(unit_make_cubic_interpolation src/unit_make_cubic_interpolation.cpp)

TARGET_LINK_LIBRARIES(unit_make_cubic_interpolation ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_make_cubic_interpolation
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_make_cubic_interpolation unit_make_cubic_interpolation )