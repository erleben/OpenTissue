add_executable(unit_obsolete_gjk src/unit_gjk.cpp)

target_link_libraries(unit_obsolete_gjk
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_obsolete_gjk
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_obsolete_gjk)

