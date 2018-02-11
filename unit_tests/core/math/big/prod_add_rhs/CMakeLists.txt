INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_prod_add_rhs src/unit_prod_add_rhs.cpp)

TARGET_LINK_LIBRARIES(unit_prod_add_rhs ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_prod_add_rhs
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_prod_add_rhs unit_prod_add_rhs )

