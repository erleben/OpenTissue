add_executable(t4_gpu_scan src/application.cpp)

target_link_libraries(t4_gpu_scan
    OpenTissue)

install(
  TARGETS t4_gpu_scan
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
