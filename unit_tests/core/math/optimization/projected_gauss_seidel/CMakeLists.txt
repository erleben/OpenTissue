INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_projected_gauss_seidel src/unit_projected_gauss_seidel.cpp)

TARGET_LINK_LIBRARIES(unit_projected_gauss_seidel ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_projected_gauss_seidel
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_projected_gauss_seidel unit_projected_gauss_seidel )

