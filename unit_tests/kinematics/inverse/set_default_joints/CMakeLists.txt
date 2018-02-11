INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_set_default_joints src/unit_set_default_joints.cpp)

TARGET_LINK_LIBRARIES(unit_set_default_joints ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_set_default_joints
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_set_default_joints unit_set_default_joints )
