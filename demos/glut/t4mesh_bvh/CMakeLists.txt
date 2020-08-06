ADD_EXECUTABLE(t4mesh_bvh src/application.cpp)

TARGET_LINK_LIBRARIES(t4mesh_bvh ${OPENTISSUE_LIBS})

target_include_directories(t4mesh_bvh
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/bundled/
)

INSTALL(
  TARGETS t4mesh_bvh
  RUNTIME DESTINATION  bin/demos/glut/
  )

