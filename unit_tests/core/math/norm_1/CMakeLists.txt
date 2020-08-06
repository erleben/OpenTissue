add_executable(unit_norm_1 src/unit_norm_1.cpp)

target_link_libraries(unit_norm_1 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_norm_1
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_norm_1)
