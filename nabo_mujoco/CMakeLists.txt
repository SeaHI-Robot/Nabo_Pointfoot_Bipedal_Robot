cmake_minimum_required(VERSION 3.0)
project(test)
set(CMAKE_BUILD_TYPE "Release")
add_definitions(-std=c++11)

set(allDir "mujoco" "glfw" "nabo")
include_directories(${allDir})
link_directories(${allDir})

aux_source_directory(. srcFiles)
add_executable(main ${srcFiles})
target_link_libraries(main mujoco glfw3 nabo)

add_custom_command(TARGET main POST_BUILD COMMAND ${CMAKE_COMMAND} -E
	copy ${CMAKE_SOURCE_DIR}/mujoco/libmujoco.so
		${CMAKE_SOURCE_DIR}/glfw/libglfw3.so
		${CMAKE_SOURCE_DIR}/nabo/libnabo.so
		${CMAKE_BINARY_DIR}
)
