cmake_minimum_required (VERSION 3.16.5)

project (AccelSim)

#set flags for cmake (to show errors and which cpp version should be used)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++14")

#define projects source directory
set (source_dir "${PROJECT_SOURCE_DIR}/src/")

#add root and include directories for boost library
set(BOOST_ROOT "${PROJECT_SOURCE_DIR}/Dependencies/boost")
set(BOOST_INCLUDEDIR "${PROJECT_SOURCE_DIR}/Dependencies/boost")

#add directory for target_link_libraries to search in
link_directories("${PROJECT_SOURCE_DIR}/Dependencies/boost/lib")

#add directory where to look for header files
set (EIGEN3_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/Dependencies/eigen-3.3.7")
set (BOOST_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/Dependencies/boost/include")
include_directories ("${EIGEN3_INCLUDE_DIR}" "${BOOST_INCLUDE_DIR}")

#look for package and required components
find_package(Boost 1.72.0 REQUIRED COMPONENTS iostreams system filesystem)

#tell cmake to compile all .cpp files found in source_dir
file (GLOB source_files "${source_dir}/*.cpp")

#define executable name and output directory
add_executable (AccelSim ${source_files})

#add libraries to be linked (without "lib" at beginning and ".so" at end of filename)
target_link_libraries(AccelSim boost_iostreams boost_system boost_filesystem util)


