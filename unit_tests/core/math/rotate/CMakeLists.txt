add_executable(unit_rotate src/unit_rotate.cpp)

target_link_libraries(unit_rotate 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_rotate
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_rotate)
