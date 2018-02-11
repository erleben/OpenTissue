INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_ik_io src/unit_ik_io.cpp)

TARGET_LINK_LIBRARIES(unit_ik_io ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_ik_io
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_ik_io unit_ik_io )
