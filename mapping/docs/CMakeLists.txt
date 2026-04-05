# Standalone docs-only CMake project.
# Mirrors the docs target in the main CMakeLists.txt but requires
# nothing beyond CMake, Doxygen, and Graphviz.
cmake_minimum_required(VERSION 3.16)
project(mapping-docs NONE)          # NONE — no compiler needed

find_package(Doxygen REQUIRED)

include(FetchContent)
FetchContent_Declare(doxygen-awesome-css
  URL https://github.com/jothepro/doxygen-awesome-css/archive/refs/heads/main.zip
)
FetchContent_MakeAvailable(doxygen-awesome-css)
FetchContent_GetProperties(doxygen-awesome-css SOURCE_DIR AWESOME_CSS_DIR)

# Doxyfile.in uses @CMAKE_CURRENT_SOURCE_DIR@ for source paths.
# Resolve the mapping/ root (one level up) so the template produces the
# same paths as the main build.
get_filename_component(CMAKE_CURRENT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

configure_file(Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

add_custom_target(docs ALL
  COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating documentation with Doxygen"
)
