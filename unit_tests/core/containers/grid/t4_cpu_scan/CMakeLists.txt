ADD_EXECUTABLE(unit_t4_cpu_scan src/unit_t4_cpu_scan.cpp)

TARGET_LINK_LIBRARIES(unit_t4_cpu_scan ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_t4_cpu_scan
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_t4_cpu_scan unit_t4_cpu_scan )
