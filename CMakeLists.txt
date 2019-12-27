project(cbs)

cmake_minimum_required (VERSION 2.6)

#set(CMAKE_CXX_STANDARD 11)
#SET(CMAKE_CXX_COMPILER "/usr/bin/g++")

#SET (CMAKE_LINKER  "/usr/bin/ld")

#SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall --std=c++0x")

include_directories("../inc")
file(GLOB SOURCES "../*src/*.cpp")



# Find Boost
SET(BOOST_ROOT "D://boost//boost_1_61_0")
SET(BOOST_LIBRARYDIR "D://boost//boost_1_61_0//stage//x64//lib")
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )


add_executable(cbs ${SOURCES} )
target_link_libraries(cbs ${Boost_LIBRARIES})
