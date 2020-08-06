add_executable(unit_polymesh src/unit_polymesh.cpp)

target_link_libraries(unit_polymesh
  PRIVATE
#       OpenGL::OpenGL
      Qhull::libqhull
      Boost::unit_test_framework
      OpenTissue
)

target_include_directories(unit_polymesh
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/bundled/
)

install(
  TARGETS unit_polymesh
  RUNTIME DESTINATION  bin/units
  )


ot_add_test(unit_polymesh)
