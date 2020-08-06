add_executable(unit_trimesh src/unit_trimesh.cpp)

target_link_libraries(unit_trimesh
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
      Qhull::libqhull
)

install(
  TARGETS unit_trimesh
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_trimesh)

