ADD_EXECUTABLE(versatile src/application.cpp)

TARGET_LINK_LIBRARIES(versatile ${OPENTISSUE_LIBS})

target_include_directories(versatile
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/bundled/
)


INSTALL(
  TARGETS versatile
  RUNTIME DESTINATION  bin/demos/glut/
  )



