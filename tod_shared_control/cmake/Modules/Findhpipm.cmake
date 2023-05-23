find_path(hpipm_INCLUDE_DIR
  NAMES
    hpipm_common.h
  PATH_SUFFIXES
  NO_DEFAULT_PATH
  PATHS
    /home/$ENV{USER}/acados/include/hpipm/include/
    /usr/include/
    /acados/include/hpipm/include/
  DOC "hpipm include directory"
)
mark_as_advanced(hpipm_INCLUDE_DIR)

find_library(hpipm_LIBRARY
  NAMES
    hpipm
  PATH_SUFFIXEs
  NO_DEFAULT_PATH
  PATHS
    /home/$ENV{USER}/acados/lib/
    /usr/lib/
    /acados/lib/
  DOC "hpipm API library")
mark_as_advanced(hpipm_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(hpipm
  REQUIRED_VARS hpipm_LIBRARY hpipm_INCLUDE_DIR)

if (hpipm_FOUND)
  set(hpipm_INCLUDE_DIRS "${hpipm_INCLUDE_DIR}")
  set(hpipm_LIBRARIES "${hpipm_LIBRARY}")
  if (NOT TARGET hpipm)
    add_library(hpipm UNKNOWN IMPORTED)
    set_target_properties(hpipm PROPERTIES
      IMPORTED_LOCATION "${hpipm_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${hpipm_INCLUDE_DIR}")
  endif ()
endif ()
