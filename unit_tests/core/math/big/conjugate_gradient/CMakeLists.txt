include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_conjugate_gradient src/unit_conjugate_gradient.cpp)

target_link_libraries(unit_conjugate_gradient 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_conjugate_gradient
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_conjugate_gradient)

