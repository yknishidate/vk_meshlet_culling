cmake_minimum_required(VERSION 3.16)

project(VkMeshletCulling LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

find_package(meshoptimizer CONFIG REQUIRED)
set(REACTIVE_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(Reactive)

file(GLOB_RECURSE sources src/*.cpp)
file(GLOB_RECURSE headers src/*.hpp)
file(GLOB_RECURSE shaders shader/*)
add_executable(${PROJECT_NAME} ${sources} ${headers} ${shaders})
source_group("Shader Files" FILES ${shaders})

target_link_libraries(${PROJECT_NAME} PRIVATE 
    Reactive
    meshoptimizer::meshoptimizer
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}/Reactive/source
    ${PROJECT_SOURCE_DIR}/src
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    "SHADER_DIR=std::string{\"${CMAKE_CURRENT_SOURCE_DIR}/shader/\"}"
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    "ASSET_DIR=std::string{\"${PROJECT_SOURCE_DIR}/Reactive/asset/\"}"
)
