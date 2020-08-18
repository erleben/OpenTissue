add_executable(unit_heap src/unit_heap.cpp)

target_link_libraries(unit_heap
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_heap
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_heap)
