# - Try to find Eigen3
# Once done this will define
#  EIGEN3_FOUND - System has Eigen3
#  EIGEN3_INCLUDE_DIRS - The Eigen3 include directories
#  EIGEN3_DEFINITIONS - Compiler switches required for using Eigen3

find_package(PkgConfig)
pkg_check_modules(PC_EIGEN3 QUIET eigen3)
set(EIGEN3_DEFINITIONS ${PC_EIGEN3_CFLAGS_OTHER})

find_path(EIGEN3_INCLUDE_DIR Eigen/Core
          HINTS ${PC_EIGEN3_INCLUDEDIR} ${PC_EIGEN3_INCLUDE_DIRS}
          PATH_SUFFIXES eigen3 )

set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set EIGEN3_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Eigen3  DEFAULT_MSG
                                  EIGEN3_INCLUDE_DIR)

mark_as_advanced(EIGEN3_INCLUDE_DIR)
