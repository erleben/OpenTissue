##################################################################################################
#
# Find module for Triangle.
#
# Input variables:
#
# - Triangle_ROOT (optional) - Stardard CMake path search variable.
# - You can also set the environment variable Triangle_ROOT variable and
#   cmake will automatically look in there.
#
# Output variables:
#
# - Triangle_FOUND: Boolean that indicates if the package was found
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the :prop_tgt:`IMPORTED` targets:
#
# ``Triangle``
#  Defined to the platform-specific Triangle library.
#
# Example usage:
#
#  find_package(Triangle)
#  if(NOT Triangle_FOUND)
#    # Error handling
#  endif()
#
#  target_link_libraries(my_target Triangle)
#
##################################################################################################

# Find headers and libraries
find_path(Triangle_INCLUDE_DIR NAMES triangle.h PATH_SUFFIXES Triangle)

find_library(Triangle_LIBRARY_RELEASE  NAMES Triangle)
find_library(Triangle_LIBRARY_DEBUG  NAMES Triangle_d)

if(Triangle_LIBRARY_RELEASE)
  set(Triangle_LIBRARIES ${Triangle_LIBRARY_RELEASE})
endif()

if(Triangle_LIBRARY_DEBUG)
  set(Triangle_LIBRARIES ${Triangle_LIBRARIES} ${Triangle_LIBRARY_DEBUG})
endif()

# Output variables generation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Triangle DEFAULT_MSG Triangle_LIBRARIES
                                                       Triangle_INCLUDE_DIR)

if(Triangle_FOUND)
  if(NOT TARGET Triangle)
    add_library(Triangle UNKNOWN IMPORTED)
    if(EXISTS ${Triangle_LIBRARY_RELEASE})
      set_property(TARGET Triangle APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(Triangle PROPERTIES MASP_IMPORTED_CONFIG_RELEASE Release
        IMPORTED_LOCATION_RELEASE "${Triangle_LIBRARY_RELEASE}")
    endif()

    if(EXISTS ${Triangle_LIBRARY_DEBUG})
      set_property(TARGET Triangle APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(Triangle PROPERTIES MASP_IMPORTED_CONFIG_DEBUG Debug
        IMPORTED_LOCATION_RELEASE "${Triangle_LIBRARY_DEBUG}")
    endif()

    set_target_properties(Triangle PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${Triangle_INCLUDE_DIR})
  endif()
endif()

mark_as_advanced(Triangle_INCLUDE_DIR
                 Triangle_LIBRARY_RELEASE
                 Triangle_LIBRARY_DEBUG
                 Triangle_LIBRARIES
)
