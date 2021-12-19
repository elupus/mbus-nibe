

cmake_minimum_required (VERSION 3.14)

project (mbus-nibe VERSION 0.1.0)

add_subdirectory(m-bus)

add_executable(mbus-nibe)
target_sources(mbus-nibe PRIVATE src/main.c)
target_link_libraries(mbus-nibe PRIVATE mbus)
