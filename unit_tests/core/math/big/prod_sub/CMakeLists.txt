INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_prod_sub src/unit_prod_sub.cpp)

TARGET_LINK_LIBRARIES(unit_prod_sub ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_prod_sub
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_prod_sub unit_prod_sub )


