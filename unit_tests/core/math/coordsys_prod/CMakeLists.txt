add_executable(unit_coordsys_prod src/unit_coordsys_prod.cpp)

target_link_libraries(unit_coordsys_prod 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_coordsys_prod
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_coordsys_prod)
