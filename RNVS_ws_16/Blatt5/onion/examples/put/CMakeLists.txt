include_directories (${PROJECT_SOURCE_DIR}/src/) 

add_executable(put put.c)
target_link_libraries(put onion)
