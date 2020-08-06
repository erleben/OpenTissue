##################################################################################################
#
# Find module for TinyXML.
#
# Input variables:
#
# - TinyXML_ROOT (optional) - Stardard CMake path search variable.
# - You can also set the environment variable TinyXML_ROOT variable and
#   cmake will automatically look in there.
#
# Output variables:
#
# - TinyXML_FOUND: Boolean that indicates if the package was found
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the :prop_tgt:`IMPORTED` targets:
#
# ``TinyXML``
#  Defined to the platform-specific TinyXML library.
#
# Example usage:
#
#  find_package(TinyXML)
#  if(NOT TinyXML_FOUND)
#    # Error handling
#  endif()
#
#  target_link_libraries(my_target TinyXML)
#
##################################################################################################

# Find headers and libraries
find_path(TinyXML_INCLUDE_DIR NAMES tinyxml.h PATH_SUFFIXES TinyXML)

find_library(TinyXML_LIBRARY_RELEASE  NAMES TinyXML)
find_library(TinyXML_LIBRARY_DEBUG  NAMES TinyXML_d)

if(TinyXML_LIBRARY_RELEASE)
  set(TinyXML_LIBRARIES ${TinyXML_LIBRARY_RELEASE})
endif()

if(TinyXML_LIBRARY_DEBUG)
  set(TinyXML_LIBRARIES ${TinyXML_LIBRARIES} ${TinyXML_LIBRARY_DEBUG})
endif()

# Output variables generation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TinyXML DEFAULT_MSG TinyXML_LIBRARIES
                                                      TinyXML_INCLUDE_DIR)

if(TinyXML_FOUND)
  if(NOT TARGET TinyXML)
    add_library(TinyXML UNKNOWN IMPORTED)
    if(EXISTS ${TinyXML_LIBRARY_RELEASE})
      set_property(TARGET TinyXML APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(TinyXML PROPERTIES MASP_IMPORTED_CONFIG_RELEASE Release
        IMPORTED_LOCATION_RELEASE "${TinyXML_LIBRARY_RELEASE}")
    endif()

    if(EXISTS ${TinyXML_LIBRARY_DEBUG})
      set_property(TARGET TinyXML APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(TinyXML PROPERTIES MASP_IMPORTED_CONFIG_DEBUG Debug
        IMPORTED_LOCATION_RELEASE "${TinyXML_LIBRARY_DEBUG}")

    endif()

    set_target_properties(TinyXML PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES ${TinyXML_INCLUDE_DIR}
      INTERFACE_COMPILE_DEFINITIONS TIXML_USE_STL
    )
  endif()
endif()

mark_as_advanced(TinyXML_INCLUDE_DIR
                 TinyXML_LIBRARY_RELEASE
                 TinyXML_LIBRARY_DEBUG
                 TinyXML_LIBRARIES
)
