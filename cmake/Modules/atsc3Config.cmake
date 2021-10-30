find_package(PkgConfig)

PKG_CHECK_MODULES(PC_ATSC3 atsc3)

FIND_PATH(
    ATSC3_INCLUDE_DIRS
    NAMES atsc3/api.h
    HINTS $ENV{ATSC3_DIR}/include
        ${PC_ATSC3_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    ATSC3_LIBRARIES
    NAMES gnuradio-atsc3
    HINTS $ENV{ATSC3_DIR}/lib
        ${PC_ATSC3_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/atsc3Target.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ATSC3 DEFAULT_MSG ATSC3_LIBRARIES ATSC3_INCLUDE_DIRS)
MARK_AS_ADVANCED(ATSC3_LIBRARIES ATSC3_INCLUDE_DIRS)
