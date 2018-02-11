INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_prod_row src/unit_prod_row.cpp)

TARGET_LINK_LIBRARIES(unit_prod_row ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_prod_row
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_prod_row unit_prod_row )

