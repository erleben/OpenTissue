include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_non_smooth_newton src/unit_non_smooth_newton.cpp)

target_link_libraries(unit_non_smooth_newton 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_non_smooth_newton
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_non_smooth_newton)

