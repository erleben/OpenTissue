##################################################################################################
#
# Find module for Qhull.
#
# Input variables:
#
# - Qhull_ROOT (optional) - Stardard CMake path search variable.
# - You can also set the environment variable Qhull_ROOT variable and
#   cmake will automatically look in there.
#
# Output variables:
#
# - Qhull_FOUND: Boolean that indicates if the package was found
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the :prop_tgt:`IMPORTED` targets:
#
# ``Qhull::libqhull``
#  Defined to the platform-specific Qhull library.
#
# Example usage:
#
#  find_package(Qhull)
#  if(NOT Qhull_FOUND)
#    # Error handling
#  endif()
#
#  target_link_libraries(my_target Qhull::libqhull)
#
##################################################################################################

# Find headers and libraries
find_path(Qhull_INCLUDE_DIR NAMES libqhull.h PATH_SUFFIXES libqhull)

find_library(Qhull_LIBRARY_RELEASE  NAMES qhull)
find_library(Qhull_LIBRARY_DEBUG  NAMES qhull_d)

if(Qhull_LIBRARY_RELEASE)
  set(Qhull_LIBRARIES ${Qhull_LIBRARY_RELEASE})
endif()

if(Qhull_LIBRARY_DEBUG)
  set(Qhull_LIBRARIES ${Qhull_LIBRARIES} ${Qhull_LIBRARY_DEBUG})
endif()

# Output variables generation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Qhull DEFAULT_MSG Qhull_LIBRARIES
                                                    Qhull_INCLUDE_DIR)

if(Qhull_FOUND)
  if(NOT TARGET Qhull)
    add_library(Qhull::libqhull UNKNOWN IMPORTED)
    if(EXISTS ${Qhull_LIBRARY_RELEASE})
      set_property(TARGET Qhull::libqhull APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(Qhull::libqhull PROPERTIES MASP_IMPORTED_CONFIG_RELEASE Release
        IMPORTED_LOCATION_RELEASE "${Qhull_LIBRARY_RELEASE}")
    endif()

    if(EXISTS ${Qhull_LIBRARY_DEBUG})
      set_property(TARGET Qhull::libqhull APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(Qhull::libqhull PROPERTIES MASP_IMPORTED_CONFIG_DEBUG Debug
        IMPORTED_LOCATION_RELEASE "${Qhull_LIBRARY_DEBUG}")
    endif()

    set_target_properties(Qhull::libqhull PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${Qhull_INCLUDE_DIR})
  endif()
endif()

mark_as_advanced(Qhull_INCLUDE_DIR
                 Qhull_LIBRARY_RELEASE
                 Qhull_LIBRARY_DEBUG
                 Qhull_LIBRARIES
)

