include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_shur_system src/unit_shur_system.cpp)

target_link_libraries(unit_shur_system 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_shur_system
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_shur_system)

