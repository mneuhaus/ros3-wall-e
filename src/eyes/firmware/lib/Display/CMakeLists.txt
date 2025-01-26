# Create Display library
add_library(Display
  Display.c
)

target_link_libraries(Display PUBLIC
  Config
)

target_include_directories(Display PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../Config
)
