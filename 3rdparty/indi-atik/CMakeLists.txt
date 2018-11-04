cmake_minimum_required(VERSION 2.6)
PROJECT(atik_ccd CXX C)

set (POSITION_INDEPENDENT_CODE OFF)
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake_modules/")

include(GNUInstallDirs)

set (ATIK_VERSION_MAJOR 1)
set (ATIK_VERSION_MINOR 0)

find_package(CFITSIO REQUIRED)
find_package(INDI REQUIRED)
find_package(ZLIB REQUIRED)
find_package(USB1 REQUIRED)
find_package(Threads REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_atik_ccd.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_atik_ccd.xml )

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/libatik/include)
include_directories( ${INDI_INCLUDE_DIR})
include_directories( ${CFITSIO_INCLUDE_DIR})

include(CMakeCommon)

# Need to disable position independent code to link with libatik
string(REPLACE "-fPIE" "-no-pie" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
string(REPLACE "-pie" "-no-pie" CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS})

if (APPLE)
    set(ATIKLIB ${CMAKE_CURRENT_SOURCE_DIR}/libatikccd/lib/macOS/libatikccd.bin)
    message("-- Using Mac static ATIK library")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "armv+")
    set(ATIKLIB ${CMAKE_CURRENT_SOURCE_DIR}/libatikccd/lib/Linux/arm/libatikccd.bin)
    message("-- Using armv6 static ATIK library")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(ATIKLIB ${CMAKE_CURRENT_SOURCE_DIR}/libatikccd/lib/Linux/arm64/libatikccd.bin)
    message("-- Using armv8 static ATIK library")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(ATIKLIB ${CMAKE_CURRENT_SOURCE_DIR}/libatikccd/lib/Linux/x64/libatikccd.bin)
    message("-- Using 64 bit static ATIK library")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "i[3-6]86")
    set(ATIKLIB ${CMAKE_CURRENT_SOURCE_DIR}/libatikccd/lib/Linux/x86/libatikccd.bin)
    message("-- Using 32 bit static ATIK library")
endif ()

############# ATIK CCD ###############
set(atikccd_SRCS
	${CMAKE_CURRENT_SOURCE_DIR}/atik_ccd.cpp
)

add_executable(indi_atik2_ccd ${atikccd_SRCS})

target_link_libraries(indi_atik2_ccd ${INDI_LIBRARIES} ${CFITSIO_LIBRARIES} ${ZLIB_LIBRARY} ${ATIKLIB} ${USB1_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

install(TARGETS indi_atik2_ccd RUNTIME DESTINATION bin)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_atik_ccd.xml DESTINATION ${INDI_DATA_DIR})
