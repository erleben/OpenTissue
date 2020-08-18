include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_partition_vector src/unit_partition_vector.cpp)

target_link_libraries(unit_partition_vector 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_partition_vector
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_partition_vector)

