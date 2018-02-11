ADD_EXECUTABLE(unit_scan_conversion src/unit_scan_conversion.cpp)

TARGET_LINK_LIBRARIES(unit_scan_conversion ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_scan_conversion 
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_scan_conversion unit_scan_conversion )

