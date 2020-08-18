add_executable(unit_dynamic_table_dispatcher src/unit_dynamic_table_dispatcher.cpp)

target_link_libraries(unit_dynamic_table_dispatcher
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_dynamic_table_dispatcher
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_dynamic_table_dispatcher)
