INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_chain src/unit_chain.cpp)

TARGET_LINK_LIBRARIES(unit_chain ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_chain
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_chain unit_chain )
