cmake_minimum_required(VERSION 3.12)

set(INCLUDE_DIRS)
function(add_include_dir IDENTIFIER)
	set(PRETTYNAME ${IDENTIFIER})
	set(ARGV ${ARGN})
	list(LENGTH ARGV ARGC)
	if(${ARGC} GREATER 0)
		list(GET ARGV 0 PRETTYNAME)
	endif()
	string(TOUPPER ${IDENTIFIER} UIDENTIFIER)

	set(${DEFAULT_DIR} "")
	set(DEPENDENCY_${UIDENTIFIER}_INCLUDE ${DEFAULT_DIR} CACHE PATH "Path to ${PRETTYNAME} include directory.")
	set(INCLUDE_DIRS ${INCLUDE_DIRS} DEPENDENCY_${UIDENTIFIER}_INCLUDE PARENT_SCOPE)
endfunction(add_include_dir)

set(LIBRARIES)
function(add_external_library IDENTIFIER)
	set(PRETTYNAME ${IDENTIFIER})
	set(ARGV ${ARGN})
	list(LENGTH ARGV ARGC)
	if(${ARGC} GREATER 0)
		list(GET ARGV 0 PRETTYNAME)
	endif()
	string(TOUPPER ${IDENTIFIER} UIDENTIFIER)

	set(DEPENDENCY_${UIDENTIFIER}_LIBRARY "" CACHE FILEPATH "Path to ${PRETTYNAME} library.")
	set(LIBRARIES ${LIBRARIES} DEPENDENCY_${UIDENTIFIER}_LIBRARY PARENT_SCOPE)
endfunction(add_external_library)

function(link_external_library IDENTIFIER)
	set(PRETTYNAME ${IDENTIFIER})
	set(ARGV ${ARGN})
	list(LENGTH ARGV ARGC)
	if(${ARGC} GREATER 0)
		list(GET ARGV 0 PRETTYNAME)
	endif()
	string(TOUPPER ${IDENTIFIER} UIDENTIFIER)

	set(${DEFAULT_DIR} "")
	set(DEPENDENCY_${UIDENTIFIER}_INCLUDE ${DEFAULT_DIR} CACHE PATH "Path to ${PRETTYNAME} include directory.")
	set(INCLUDE_DIRS ${INCLUDE_DIRS} DEPENDENCY_${UIDENTIFIER}_INCLUDE PARENT_SCOPE)

	set(DEPENDENCY_${UIDENTIFIER}_LIBRARY "" CACHE FILEPATH "Path to ${PRETTYNAME} library.")
	set(LIBRARIES ${LIBRARIES} DEPENDENCY_${UIDENTIFIER}_LIBRARY PARENT_SCOPE)
endfunction(link_external_library)

set(DEFINITIONS)
function(add_def DEF)
	set(DEFINITIONS ${DEFINITIONS} ${DEF} PARENT_SCOPE)
endfunction(add_def)

set(UNIRENDER_CYCLES_ENABLE_DEPENDENCIES 1 CACHE BOOL "Enable project dependencies?")

##### CONFIGURATION #####

set(PROJ_NAME UniRender_cycles)

project(${PROJ_NAME} CXX)

set(CMAKE_CXX_STANDARD 20)
set(ENABLE_STATIC_LIBRARY_FLAG 0)

link_external_library(util_image)
link_external_library(util_raytracing)
link_external_library(sharedutils)
link_external_library(mathutil)
link_external_library(vfilesystem)
link_external_library(cycles_tbb)
link_external_library(util_udm)
link_external_library(lz4)
add_include_dir(cycles_glog)
add_include_dir(gflags)

add_include_dir(cycles)
add_include_dir(cycles_atomic)
add_include_dir(cycles_pugixml)
add_include_dir(cycles_openimageio)
add_include_dir(cycles_openimagedenoise)
add_include_dir(cycles_openexr)
add_include_dir(cycles_embree)
add_include_dir(cycles_osl)
add_include_dir(glm)
add_include_dir(glew)
add_include_dir(tiff)
add_include_dir(lpng)
add_include_dir(zlib)
add_include_dir(jpeg)

# Required for color management
link_external_library(util_ocio)
#

set(DEPENDENCY_CYCLES_LIBRARY_LOCATION "" CACHE FILEPATH "Path to cycles libraries.")
set(DEPENDENCY_CYCLES_DEPENDENCIES_LOCATION "" CACHE FILEPATH "Path to cycles dependencies.")
set(DEPENDENCY_OPENEXR_LIBRARY_LOCATION "" CACHE FILEPATH "Path to openexr libraries.")
add_def(OIIO_STATIC_BUILD)
add_def(GLEW_STATIC)
add_def(WITH_SYSTEM_PUGIXML)
add_def(OIIO_STATIC_BUILD)
add_def(OIIO_STATIC_DEFINE)
add_def(WITH_OSL)
add_def(OSL_STATIC_BUILD)
add_def(OSL_STATIC_LIBRARY)
add_def(WITH_OPTIX)
add_def(WITH_EMBREE)
add_def(WITH_OPENSUBDIV)
add_def(WITH_OPENIMAGEDENOISE)
add_def(OIDN_STATIC_LIB)
if(WIN32)
	add_def(WITH_CYCLES_LOGGING)
endif()
add_def(GLOG_NO_ABBREVIATED_SEVERITIES)
add_def(GOOGLE_GLOG_DLL_DECL=__declspec\(dllexport\))
add_def("CCL_NAMESPACE_BEGIN=namespace ccl {")
add_def("CCL_NAMESPACE_END=}")

add_include_dir(openexr_imath)
add_include_dir(openexr_iex)
add_include_dir(openexr_ilmthread)
add_include_dir(openexr)
add_include_dir(openexr_core)
add_include_dir(openexr_util)

##### CONFIGURATION #####

set(LIB_TYPE SHARED)

foreach(def IN LISTS DEFINITIONS)
	add_definitions(-D${def})
endforeach(def)

function(def_vs_filters FILE_LIST)
	foreach(source IN LISTS FILE_LIST)
	    get_filename_component(source_path "${source}" PATH)
	    string(REPLACE "${CMAKE_CURRENT_LIST_DIR}" "" source_path_relative "${source_path}")
	    string(REPLACE "/" "//" source_path_msvc "${source_path_relative}")
	    source_group("${source_path_msvc}" FILES "${source}")
	endforeach()
endfunction(def_vs_filters)

file(GLOB_RECURSE SRC_FILES
    "${CMAKE_CURRENT_LIST_DIR}/src/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/src/*.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/*.c"
    "${CMAKE_CURRENT_LIST_DIR}/include/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/*.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/cycles_interface/src/icycles.cpp"
)
add_library(${PROJ_NAME} ${LIB_TYPE} ${SRC_FILES})
if(WIN32)
	target_compile_options(${PROJ_NAME} PRIVATE /wd4251)
	target_compile_options(${PROJ_NAME} PRIVATE /wd4996)
endif()
def_vs_filters("${SRC_FILES}")

foreach(LIB IN LISTS LIBRARIES)
	target_link_libraries(${PROJ_NAME} ${${LIB}})
endforeach(LIB)

target_include_directories(${PROJ_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(${PROJ_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/cycles_interface/include/)
target_include_directories(${PROJ_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

foreach(INCLUDE_PATH IN LISTS INCLUDE_DIRS)
	target_include_directories(${PROJ_NAME} PRIVATE ${${INCLUDE_PATH}})
endforeach(INCLUDE_PATH)

set(TARGET_PROPERTIES LINKER_LANGUAGE CXX)
if(DEFINED PRECOMPILED_HEADER)
	include(PrecompiledHeader.cmake)
	add_precompiled_header(${PROJ_NAME} "src/${PRECOMPILED_HEADER}.h" c++17 FORCEINCLUDE)
endif()
set_target_properties(${PROJ_NAME} PROPERTIES ${TARGET_PROPERTIES})

if(WIN32)
	target_link_libraries(${PROJ_NAME}
		opengl32
		glu32
		version
		shlwapi
	)
endif()

add_subdirectory(cycles_interface)

if(${UNIRENDER_CYCLES_ENABLE_DEPENDENCIES})
	add_dependencies(${PROJ_NAME} util_raytracing)
endif()
add_dependencies(${PROJ_NAME} icycles)

if(WIN32)
	set(LIB_PREFIX "")
	set(LIB_EXT ".lib")
else()
	set(LIB_PREFIX "lib")
	set(LIB_EXT ".a")
endif()

set(DEPENDENCY_CYCLES_UTIL_LIBRARY ${DEPENDENCY_CYCLES_LIBRARY_LOCATION}/${LIB_PREFIX}cycles_util${LIB_EXT} CACHE PATH "Path to library." FORCE)

SET(CYCLES_LIBRARIES
	${DEPENDENCY_CYCLES_UTIL_LIBRARY}
)

target_link_libraries(${PROJ_NAME} ${CYCLES_LIBRARIES})
if(UNIX)
	# Link a few more times on Linux so we don't have to bother trying to find the right linking order...
	target_link_libraries(${PROJ_NAME} ${CYCLES_LIBRARIES})
	
	foreach(LIB IN LISTS LIBRARIES)
		target_link_libraries(${PROJ_NAME} ${${LIB}})
	endforeach(LIB)
	
	target_link_libraries(${PROJ_NAME} ${CYCLES_LIBRARIES})
endif()