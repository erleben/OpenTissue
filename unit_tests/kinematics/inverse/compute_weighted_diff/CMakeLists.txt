include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_compute_weighted_diff src/unit_compute_weighted_diff.cpp)

target_link_libraries(unit_compute_weighted_diff 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_compute_weighted_diff
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_compute_weighted_diff)
