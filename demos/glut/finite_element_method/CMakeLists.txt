ADD_EXECUTABLE(finite_element_method src/application.cpp)

TARGET_LINK_LIBRARIES(finite_element_method ${OPENTISSUE_LIBS})

target_include_directories(finite_element_method
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/bundled/
)

INSTALL(
  TARGETS finite_element_method
  RUNTIME DESTINATION  bin/demos/glut/
  )
