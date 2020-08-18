add_executable(unit_t4mesh_xml src/unit_t4mesh_xml.cpp)

target_link_libraries(unit_t4mesh_xml
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
      TinyXML
)

install(
  TARGETS unit_t4mesh_xml
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_t4mesh_xml)


