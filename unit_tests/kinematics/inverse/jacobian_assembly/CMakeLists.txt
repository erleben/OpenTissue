INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_jacobian_assembly src/unit_jacobian_assembly.cpp)

TARGET_LINK_LIBRARIES(unit_jacobian_assembly ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_jacobian_assembly
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_jacobian_assembly unit_jacobian_assembly )
