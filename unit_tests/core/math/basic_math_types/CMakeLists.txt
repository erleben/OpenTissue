add_executable(unit_basic_math_types src/unit_basic_math_types.cpp)

target_link_libraries(unit_basic_math_types 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_basic_math_types
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_basic_math_types)
