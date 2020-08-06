##################################################################################################
#
# Find module for TetGen.
#
# Input variables:
#
# - TetGen_ROOT (optional) - Stardard CMake path search variable.
# - You can also set the environment variable TetGen_ROOT variable and
#   cmake will automatically look in there.
#
# Output variables:
#
# - TetGen_FOUND: Boolean that indicates if the package was found
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the :prop_tgt:`IMPORTED` targets:
#
# ``TetGen``
#  Defined to the platform-specific TetGen library.
#
# Example usage:
#
#  find_package(TetGen)
#  if(NOT TetGen_FOUND)
#    # Error handling
#  endif()
#
#  target_link_libraries(my_target TetGen)
#
##################################################################################################

# Find headers and libraries
find_path(TetGen_INCLUDE_DIR NAMES tetgen.h)

find_library(TetGen_LIBRARY_RELEASE  NAMES TetGen)
find_library(TetGen_LIBRARY_DEBUG  NAMES TetGen_d)

if(TetGen_LIBRARY_RELEASE)
  set(TetGen_LIBRARIES ${TetGen_LIBRARY_RELEASE})
endif()

if(TetGen_LIBRARY_DEBUG)
  set(TetGen_LIBRARIES ${TetGen_LIBRARIES} ${TetGen_LIBRARY_DEBUG})
endif()

# Output variables generation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TetGen DEFAULT_MSG TetGen_LIBRARIES
                                                      TetGen_INCLUDE_DIR)

if(TetGen_FOUND)
  if(NOT TARGET TetGen)
    add_library(TetGen UNKNOWN IMPORTED)
    if(EXISTS ${TetGen_LIBRARY_RELEASE})
      set_property(TARGET TetGen APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(TetGen PROPERTIES MASP_IMPORTED_CONFIG_RELEASE Release
        IMPORTED_LOCATION_RELEASE "${TetGen_LIBRARY_RELEASE}")
    endif()

    if(EXISTS ${TetGen_LIBRARY_DEBUG})
      set_property(TARGET TetGen APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(TetGen PROPERTIES MASP_IMPORTED_CONFIG_DEBUG Debug
        IMPORTED_LOCATION_RELEASE "${TetGen_LIBRARY_DEBUG}")
    endif()

    set_target_properties(TetGen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${TetGen_INCLUDE_DIR})
  endif()
endif()

mark_as_advanced(TetGen_INCLUDE_DIR
                 TetGen_LIBRARY_RELEASE
                 TetGen_LIBRARY_DEBUG
                 TetGen_LIBRARIES
)
