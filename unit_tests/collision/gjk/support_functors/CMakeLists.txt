add_executable(unit_support_functors src/unit_support_functors.cpp)

target_link_libraries(unit_support_functors
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_support_functors
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_support_functors)

