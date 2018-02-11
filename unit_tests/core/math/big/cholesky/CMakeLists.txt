INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_cholesky src/unit_cholesky.cpp)

TARGET_LINK_LIBRARIES(unit_cholesky ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}) 

INSTALL(
  TARGETS unit_cholesky
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_cholesky unit_cholesky )

