cmake_minimum_required(VERSION 3.16)
project(LegSA_GINS VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE LEGSA_SOURCES CONFIGURE_DEPENDS src/core/*.cpp)
add_library(legsa_core STATIC ${LEGSA_SOURCES})
target_include_directories(legsa_core PUBLIC include)
add_executable(legsa_gins src/main.cpp)
target_link_libraries(legsa_gins PRIVATE legsa_core)
install(TARGETS legsa_gins RUNTIME DESTINATION bin)
