add_executable(unit_contiguous_interval src/unit_contiguous_interval.cpp)

target_link_libraries(unit_contiguous_interval 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_contiguous_interval
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_contiguous_interval)
