ADD_EXECUTABLE(unit_kmeans src/unit_kmeans.cpp)

TARGET_LINK_LIBRARIES(unit_kmeans ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_kmeans
  RUNTIME DESTINATION  bin/units
  )


ADD_TEST( unit_kmeans unit_kmeans )
