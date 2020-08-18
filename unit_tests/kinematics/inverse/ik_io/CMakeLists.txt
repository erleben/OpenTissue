include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_ik_io src/unit_ik_io.cpp)

target_link_libraries(unit_ik_io
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
    TinyXML
)

install(
  TARGETS unit_ik_io
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_ik_io)
