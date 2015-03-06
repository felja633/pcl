###############################################################################
# - Try to find libfreenect2 directory
# -----------------------

find_path(LIBFREENECT2_INCLUDE_DIR
          libfreenect2.h
          PATH_SUFFIXES libfreenect2/
          )

find_library(LIBFREENECT2_LIBRARY QUIET
          NAMES libfreenect2
             )

set(LIBFREENECT2_LIBRARIES ${LIBFREENECT2_LIBRARY})
set(LIBFREENECT2_INCLUDE_DIRS ${LIBFREENECT2_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ENSENSO_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(libfreenect2 DEFAULT_MSG
                                  LIBFREENECT2_LIBRARY LIBFREENECT2_INCLUDE_DIR)

mark_as_advanced(LIBFREENECT2_INCLUDE_DIR LIBFREENECT2_LIBRARY)

if(LIBFREENECT2_FOUND)
  message(STATUS "libfreenect2 found")
endif(LIBFREENECT2_FOUND)
