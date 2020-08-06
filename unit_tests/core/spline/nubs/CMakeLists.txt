add_executable(unit_nubs src/unit_nubs.cpp)

target_link_libraries(unit_nubs 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_nubs
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_nubs)
