include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_read_dlm src/unit_read_dlm.cpp)

target_link_libraries(unit_read_dlm 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_read_dlm
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_read_dlm)

