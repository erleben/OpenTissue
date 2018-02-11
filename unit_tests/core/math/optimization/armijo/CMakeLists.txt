INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_armijo src/unit_armijo.cpp)

TARGET_LINK_LIBRARIES(unit_armijo ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_armijo
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_armijo unit_armijo )

