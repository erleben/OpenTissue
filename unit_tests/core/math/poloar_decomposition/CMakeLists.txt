add_executable(unit_polar_decomposition src/unit_poloar_decomposition.cpp)

target_link_libraries(unit_polar_decomposition 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_polar_decomposition
  RUNTIME DESTINATION  bin/units
  )


ot_add_test(unit_polar_decomposition)
