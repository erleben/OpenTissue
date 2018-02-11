INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_residual src/unit_residual.cpp)

TARGET_LINK_LIBRARIES(unit_residual ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_residual
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_residual unit_residual )

