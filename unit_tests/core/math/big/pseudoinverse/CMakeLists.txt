include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_pseudoinverse src/unit_pseudoinverse.cpp)

target_link_libraries(unit_pseudoinverse 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_pseudoinverse
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_pseudoinverse)

