project(cbs)

cmake_minimum_required (VERSION 2.6)

include_directories("inc")
file(GLOB SOURCES "src/*.cpp")

# Find Boost
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )

add_executable(cbs ${SOURCES} )
target_link_libraries(cbs ${Boost_LIBRARIES})
