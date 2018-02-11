ADD_EXECUTABLE(unit_eigen src/unit_eigen.cpp)

TARGET_LINK_LIBRARIES(unit_eigen ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_eigen
  RUNTIME DESTINATION  bin/units
  )


ADD_TEST( unit_eigen unit_eigen )
