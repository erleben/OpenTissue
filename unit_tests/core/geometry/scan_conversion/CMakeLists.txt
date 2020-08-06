add_executable(unit_scan_conversion src/unit_scan_conversion.cpp)

target_link_libraries(unit_scan_conversion
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_scan_conversion
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_scan_conversion)

