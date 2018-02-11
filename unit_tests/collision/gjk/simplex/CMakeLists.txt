ADD_EXECUTABLE(unit_simplex src/unit_simplex.cpp)

TARGET_LINK_LIBRARIES(unit_simplex ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_simplex
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_simplex unit_simplex )

