INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_joint_parameters src/unit_joint_parameters.cpp)

TARGET_LINK_LIBRARIES(unit_joint_parameters ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_joint_parameters
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_joint_parameters unit_joint_parameters )
