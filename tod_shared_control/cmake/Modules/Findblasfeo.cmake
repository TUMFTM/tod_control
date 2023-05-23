find_path(blasfeo_INCLUDE_DIR
  NAMES
    blasfeo.h
  PATH_SUFFIXES
  NO_DEFAULT_PATH
  PATHS
    /home/$ENV{USER}/acados/include/blasfeo/include/
    /usr/include/
    /acados/include/blasfeo/include/
  DOC "blasfeo include directory"
)
mark_as_advanced(blasfeo_INCLUDE_DIR)

find_library(blasfeo_LIBRARY
  NAMES
    blasfeo
  PATH_SUFFIXEs
  NO_DEFAULT_PATH
  PATHS
    /home/$ENV{USER}/acados/lib/
    /usr/lib/
    /acados/lib/
  DOC "blasfeo API library")
mark_as_advanced(blasfeo_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(blasfeo
  REQUIRED_VARS blasfeo_LIBRARY blasfeo_INCLUDE_DIR)

if (blasfeo_FOUND)
  set(blasfeo_INCLUDE_DIRS "${blasfeo_INCLUDE_DIR}")
  set(blasfeo_LIBRARIES "${blasfeo_LIBRARY}")
  if (NOT TARGET blasfeo)
    add_library(blasfeo UNKNOWN IMPORTED)
    set_target_properties(blasfeo PROPERTIES
      IMPORTED_LOCATION "${blasfeo_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${blasfeo_INCLUDE_DIR}")
  endif ()
endif ()
