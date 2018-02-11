INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_prod_add src/unit_prod_add.cpp)

TARGET_LINK_LIBRARIES(unit_prod_add ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_prod_add
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_prod_add unit_prod_add )
