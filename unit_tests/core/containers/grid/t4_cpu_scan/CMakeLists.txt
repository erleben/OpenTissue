add_executable(unit_t4_cpu_scan src/unit_t4_cpu_scan.cpp)

target_link_libraries(unit_t4_cpu_scan
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_t4_cpu_scan
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_t4_cpu_scan)
