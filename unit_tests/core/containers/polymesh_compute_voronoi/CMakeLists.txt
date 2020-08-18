add_executable(unit_polymesh_compute_voronoi src/unit_polymesh_compute_voronoi.cpp)

target_link_libraries(unit_polymesh_compute_voronoi
  PRIVATE
      Boost::unit_test_framework
      Triangle
      OpenTissue
)

target_include_directories(unit_polymesh_compute_voronoi
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/bundled/
)

install(
  TARGETS unit_polymesh_compute_voronoi
  RUNTIME DESTINATION  bin/units
  )


ot_add_test(unit_polymesh_compute_voronoi)
