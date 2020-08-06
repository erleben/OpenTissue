ADD_EXECUTABLE(t4mesh_tetgen_generator src/application.cpp)

TARGET_LINK_LIBRARIES(t4mesh_tetgen_generator ${OPENTISSUE_LIBS})

target_include_directories(t4mesh_tetgen_generator
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/bundled/
)

INSTALL(
  TARGETS t4mesh_tetgen_generator
  RUNTIME DESTINATION  bin/demos/glut/
  )
