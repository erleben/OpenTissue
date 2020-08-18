add_executable(unit_init_m_table src/unit_init_m_table.cpp)

target_link_libraries(unit_init_m_table 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_init_m_table
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_init_m_table)
