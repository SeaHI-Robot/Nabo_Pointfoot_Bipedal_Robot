# /*
# Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
# Under MIT License.

# <https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
# */

cmake_minimum_required(VERSION 2.8.12)
project(manager)
set(CMAKE_BUILD_TYPE "Release")

add_definitions(-std=c++11)
add_compile_options(-fPIC)


set(dirRbdl "third_party/rbdl")
set(dirEiQP "third_party/eiquadprog")
set(dirUtil "utility")
set(incEIGEN "third_party/eigen3")

set(dirNabo "src/robot" "src/balance" "src/plan" "src/common"  "src/manager" "src/estimate")
set(allInc ${dirUtil} ${dirRbdl} ${dirEiQP} ${incEIGEN} ${dirNabo})
set(allLib ${dirRbdl} ${dirEiQP})
include_directories(${allInc})
link_directories(${allLib})

# ================================
file(GLOB_RECURSE utilFiles ${dirUtil}/*.cpp)
add_library(util STATIC ${utilFiles})
# ================================
file(GLOB_RECURSE srcFiles "src/*.cpp")
add_library(nabo SHARED ${srcFiles})
# ================================
target_link_libraries(nabo util rbdl EiQP pthread)
# endif()
# ================================
aux_source_directory(test testFiles)
add_executable(main ${testFiles})
target_link_libraries(main nabo)
# ===============================


add_custom_command(TARGET nabo POST_BUILD COMMAND ${CMAKE_COMMAND} -E
	make_directory ${CMAKE_SOURCE_DIR}/nabo_output
)


add_custom_command(TARGET nabo POST_BUILD COMMAND ${CMAKE_COMMAND} -E
	copy ${CMAKE_BINARY_DIR}/libnabo.so
		${CMAKE_SOURCE_DIR}/src/manager/nabo.h
		${CMAKE_SOURCE_DIR}/src/manager/nabo_config.h
		${CMAKE_SOURCE_DIR}/nabo_output
)
