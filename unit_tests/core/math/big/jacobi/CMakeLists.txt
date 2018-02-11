INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_jacobi src/unit_jacobi.cpp)

TARGET_LINK_LIBRARIES(unit_jacobi ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_jacobi
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_jacobi unit_jacobi )

