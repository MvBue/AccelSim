Installation process for Boost Library on Ubuntu 18.04

MANDATORY (refere to: https://www.boost.org/doc/libs/1_72_0/more/getting_started/unix-variants.html)

1. Download boost library and extract it

2. Go into extractet folder (e.g. "..../boost_1_72_0")

3. Use command $ ./bootstrap.sh --prefix=choose/installation/folder --with-libraries=list
where: choose/installation/folder is the directory where boost should be installed to and 
where: list is a comma-separated list defining the libraries to be built. e.g: "iostreams,system,filesystem"
Executable called "b2" will be created within current directory

4. Use command $ ./b2 install
Boost libraries will be built and ALL header files are coppied to installation directory.

5. Add the following lines to the CMakeLists.txt of your project (example with iostreams,system,filesystem installed):
"#add root and include directories for boost library
set(BOOST_ROOT "${PROJECT_SOURCE_DIR}/Dependencies/boost")
set(BOOST_INCLUDEDIR "${PROJECT_SOURCE_DIR}/Dependencies/boost")

#add directory for target_link_libraries to search in
link_directories("${PROJECT_SOURCE_DIR}/Dependencies/boost/lib")

#add directory where to look for header files
set ( BOOST_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/Dependencies/boost/include" )
include_directories ( "${BOOST_INCLUDE_DIR}" )

#look for package and required components
find_package(Boost 1.72.0 REQUIRED COMPONENTS iostreams system filesystem)

#add libraries to be linked (without "lib" at beginning and ".so" at end of filename)
target_link_libraries(AccelSim boost_iostreams boost_system boost_filesystem util)"


OPTIONAL (refere to: https://www.boost.org/doc/libs/1_72_0/tools/bcp/doc/html/index.html)
The current boost installation contains many header files, which are probably unused and thus not needed by your project. To remove unneeded header files follow the following steps.

4. Go to your source project folder (containing .cpp files) 

5. Use command $ bcp --scan --boost=boost/headerfiles Main.cpp Class.cpp Class.h boost_needed_headers
where: boost/headerfiles is the include/ folder of your boost installation (e.g. "/home/myproject/dependencies/boost/include/")
This will scan your list of files (here: Main.cpp, Class.cpp and Class.h) for dependencies on the boost library and coppies all header files that your project needs to the subdirectory /boost_needed_headers.

6. Replace all files in the current /includes folder of the boost installation with the files in /boost_needed_headers.

7. DONE!


