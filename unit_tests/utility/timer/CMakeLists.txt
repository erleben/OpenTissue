add_executable(unit_timer src/unit_timer.cpp)

target_link_libraries(unit_timer 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_timer
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_timer)
