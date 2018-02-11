ADD_EXECUTABLE(unit_plane_box src/unit_plane_box.cpp)

TARGET_LINK_LIBRARIES(unit_plane_box ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_plane_box
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_plane_box unit_plane_box )
