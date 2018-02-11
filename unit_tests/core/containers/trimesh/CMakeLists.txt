ADD_EXECUTABLE(unit_trimesh src/unit_trimesh.cpp)

TARGET_LINK_LIBRARIES(unit_trimesh ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_trimesh
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_trimesh unit_trimesh )

