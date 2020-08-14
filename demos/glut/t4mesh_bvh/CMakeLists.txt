add_executable(t4mesh_bvh src/application.cpp)

target_link_libraries(t4mesh_bvh
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    ${IL_LIBRARIES}
    ${ILU_LIBRARIES}
    ${ILUT_LIBRARIES}
    Qhull::libqhull
    TinyXML
    OpenTissue
)

target_include_directories(t4mesh_bvh
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/bundled/
)

install(
  TARGETS t4mesh_bvh
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )

