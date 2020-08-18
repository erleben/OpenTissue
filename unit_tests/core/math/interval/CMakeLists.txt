add_executable(unit_interval src/unit_interval.cpp)

target_link_libraries(unit_interval 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_interval
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_interval)


