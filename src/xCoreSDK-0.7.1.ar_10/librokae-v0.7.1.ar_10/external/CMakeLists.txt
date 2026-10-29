# Third-party dependencies

# Header-only library Eigen
add_library(eigen INTERFACE)
target_compile_definitions(eigen INTERFACE EIGEN_DONT_ALIGN_STATICALLY)
target_include_directories(eigen INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_CXX_COMPILER MATCHES "aarch64")
    set(X86_64 FALSE)
	set(SUBDIR "aarch64")
else()
    set(X86_64 TRUE)
	set(SUBDIR "x86_64")
endif()

# g3log library
add_library(g3log INTERFACE)
target_compile_definitions(g3log INTERFACE)
target_include_directories(g3log INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/g3log/include>)
if(UNIX)
    target_link_libraries(g3log INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/g3log/lib/${SUBDIR}/libg3log.a
        ${CMAKE_THREAD_LIBS_INIT})
endif()

# Orocos KDL library
add_library(kdl INTERFACE)
target_compile_definitions(kdl INTERFACE)
target_include_directories(kdl INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/kdl/include>)
if(UNIX)
    target_link_libraries(kdl INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/kdl/lib/${SUBDIR}/liborocos-kdl.a
        ${CMAKE_THREAD_LIBS_INIT})
endif()
