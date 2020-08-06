add_executable(unit_tag_traits src/unit_tag_traits.cpp)

target_link_libraries(unit_tag_traits 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_tag_traits
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_tag_traits)
