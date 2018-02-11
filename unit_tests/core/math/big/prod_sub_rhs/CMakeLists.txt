INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_prod_sub_rhs src/unit_prod_sub_rhs.cpp)

TARGET_LINK_LIBRARIES(unit_prod_sub_rhs ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_prod_sub_rhs
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_prod_sub_rhs unit_prod_sub_rhs )

