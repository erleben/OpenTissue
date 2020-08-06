add_executable(unit_kmeans src/unit_kmeans.cpp)

target_link_libraries(unit_kmeans 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_kmeans
  RUNTIME DESTINATION  bin/units
  )


ot_add_test(unit_kmeans)
