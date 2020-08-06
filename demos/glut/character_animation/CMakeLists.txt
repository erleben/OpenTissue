ADD_EXECUTABLE(character_animation src/application.cpp)

TARGET_LINK_LIBRARIES(character_animation ${OPENTISSUE_LIBS})

target_include_directories(character_animation
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/bundled/TetGen/
        ${PROJECT_SOURCE_DIR}/third_party/bundled/
        ${PROJECT_SOURCE_DIR}/third_party/bundled/Triangle/
)

INSTALL(
  TARGETS character_animation
  RUNTIME DESTINATION  bin/demos/glut/
  )
