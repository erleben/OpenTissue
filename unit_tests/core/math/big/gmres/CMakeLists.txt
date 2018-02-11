INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_gmres src/unit_gmres.cpp)

TARGET_LINK_LIBRARIES(unit_gmres ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_gmres
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_gmres unit_gmres )

