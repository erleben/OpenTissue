INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_compute_inverse_D src/unit_compute_inverse_D.cpp)

TARGET_LINK_LIBRARIES(unit_compute_inverse_D ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_compute_inverse_D
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_compute_inverse_D unit_compute_inverse_D )

