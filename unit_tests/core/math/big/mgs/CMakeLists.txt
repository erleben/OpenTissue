include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_mgs src/unit_mgs.cpp)

target_link_libraries(unit_mgs 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)
install(
  TARGETS unit_mgs
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_mgs)

