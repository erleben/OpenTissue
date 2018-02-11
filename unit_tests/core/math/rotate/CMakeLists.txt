ADD_EXECUTABLE(unit_rotate src/unit_rotate.cpp)

TARGET_LINK_LIBRARIES(unit_rotate ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_rotate
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_rotate unit_rotate )