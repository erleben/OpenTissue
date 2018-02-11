ADD_EXECUTABLE(unit_heap src/unit_heap.cpp)

TARGET_LINK_LIBRARIES(unit_heap ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_heap 
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_heap unit_heap )
