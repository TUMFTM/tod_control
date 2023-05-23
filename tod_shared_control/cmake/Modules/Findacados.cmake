find_path(acados_INCLUDE_DIR
  NAMES
    acados_c
  PATH_SUFFIXES
  NO_DEFAULT_PATH
  PATHS
    /home/$ENV{USER}/acados/include/
    /usr/include/
    /acados/include/
  DOC "acados include directory"
)
mark_as_advanced(acados_INCLUDE_DIR)

find_library(acados_LIBRARY
  NAMES
    acados
  PATH_SUFFIXEs
  NO_DEFAULT_PATH
  PATHS
    /home/$ENV{USER}/acados/lib/
    /usr/lib/
    /acados/lib/
  DOC "acados API library")
mark_as_advanced(acados_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(acados
  REQUIRED_VARS acados_LIBRARY acados_INCLUDE_DIR)

if (acados_FOUND)
  set(acados_INCLUDE_DIRS "${acados_INCLUDE_DIR}")
  set(acados_LIBRARIES "${acados_LIBRARY}")
  if (NOT TARGET acados)
    add_library(acados UNKNOWN IMPORTED)
    set_target_properties(acados PROPERTIES
      IMPORTED_LOCATION "${acados_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${acados_INCLUDE_DIR}")
  endif ()
endif ()
