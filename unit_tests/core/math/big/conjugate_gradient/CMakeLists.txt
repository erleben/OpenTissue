INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_conjugate_gradient src/unit_conjugate_gradient.cpp)

TARGET_LINK_LIBRARIES(unit_conjugate_gradient ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_conjugate_gradient
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_conjugate_gradient unit_conjugate_gradient )

