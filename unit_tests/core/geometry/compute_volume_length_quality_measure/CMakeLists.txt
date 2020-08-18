add_executable(unit_compute_volume_length_quality_measure src/unit_compute_volume_length_quality_measure.cpp)

target_link_libraries(unit_compute_volume_length_quality_measure
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_compute_volume_length_quality_measure
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_compute_volume_length_quality_measure)
