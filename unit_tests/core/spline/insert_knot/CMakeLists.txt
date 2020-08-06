add_executable(unit_insert_knot src/unit_insert_knot.cpp)

target_link_libraries(unit_insert_knot 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_insert_knot
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_insert_knot)
