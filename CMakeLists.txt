project(cbs)

cmake_minimum_required (VERSION 3.17)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall")
set(CMAKE_CXX_STANDARD 17)

include_directories("inc")
file(GLOB SOURCES "src/*.cpp")

# Find Boost
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)
include_directories(
        ${Boost_INCLUDE_DIRS}
        $ENV{GUROBI_HOME}/include
        $ENV{COINOR_HOME}/dist/include
        )

link_directories(
        $ENV{GUROBI_HOME}/lib
        $ENV{COINOR_HOME}/dist/lib
        )

link_libraries(pthread  # Used by one of the other dependencies
        stdc++fs
        gurobi_c++
        gurobi81
        spdlog
        Osi
        OsiGrb
        CoinUtils
        )

add_executable(cbs ${SOURCES} )

set_property(
  TARGET cbs 
  PROPERTY COMPILE_DEFINITIONS IL_STD
  )

target_link_libraries(cbs ${Boost_LIBRARIES})
