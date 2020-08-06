add_executable(unit_make_periodic src/unit_make_periodic.cpp)

target_link_libraries(unit_make_periodic 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_make_periodic
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_make_periodic)
