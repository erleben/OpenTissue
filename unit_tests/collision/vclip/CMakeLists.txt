add_executable(unit_vclip src/unit_vclip.cpp)

target_link_libraries(unit_vclip
  PRIVATE
      Qhull::libqhull
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_vclip
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_vclip)



