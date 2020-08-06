include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_compute_generalized_minimal_map src/unit_comp_gen_min_map.cpp)

target_link_libraries(unit_compute_generalized_minimal_map 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_compute_generalized_minimal_map
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_compute_generalized_minimal_map)

