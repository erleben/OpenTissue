
set(PNG_TEST_IMAGE "https://upload.wikimedia.org/wikipedia/commons/thumb/b/ba/Pax_tux.png/170px-Pax_tux.png")

file(
  DOWNLOAD ${PNG_TEST_IMAGE} ${CMAKE_CURRENT_BINARY_DIR}/pax.png
  STATUS _status
  SHOW_PROGRESS
)

add_executable(unit_read_png src/unit_read_png.cpp)

target_link_libraries(unit_read_png
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
    PNG::PNG
)

ot_add_test(unit_read_png)
