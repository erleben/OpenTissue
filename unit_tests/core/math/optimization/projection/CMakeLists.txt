INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_projection src/unit_projection.cpp)

TARGET_LINK_LIBRARIES(unit_projection ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_projection
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_projection unit_projection )

