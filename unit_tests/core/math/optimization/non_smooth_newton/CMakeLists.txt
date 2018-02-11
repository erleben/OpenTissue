INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_non_smooth_newton src/unit_non_smooth_newton.cpp)

TARGET_LINK_LIBRARIES(unit_non_smooth_newton ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_non_smooth_newton
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_non_smooth_newton unit_non_smooth_newton )

