##################################################################################################
#
# Find module for DevIL.
#
# Input variables:
#
# - DevIL_ROOT (optional) - Stardard CMake path search variable.
# - You can also set the environment variable DevIL_ROOT variable and
#   cmake will automatically look in there.
#
# Output variables:
#
# - DevIL_FOUND: Boolean that indicates if the package was found
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the :prop_tgt:`IMPORTED` targets:
#
# ``DevIL::IL, DevIL::ILU, DevIL::ILUT``
#  Defined to the platform-specific DevIL library.
#
# Example usage:
#
#  find_package(DevIL)
#  if(NOT DevIL_FOUND)
#    # Error handling
#  endif()
#
#  target_link_libraries(my_target DevIL::IL DevIL::ILU DevIL::ILUT)
#
##################################################################################################

# If we have the conan target then use it and exit.
if(TARGET CONAN_PKG::DevIL)
  if(NOT TARGET DevIL::DevIL)
    add_library(DevIL::DevIL INTERFACE IMPORTED)
    target_link_libraries(DevIL::DevIL INTERFACE CONAN_PKG::DevIL)
  endif()
  return()
endif()

# Find headers and libraries
find_path(DevIL_INCLUDE_DIR NAMES il.h ilu.h ilut.h PATH_SUFFIXES IL)

set(IL_NAME IL)
if(MSVC)
  set(IL_NAME DevIL)
endif()

foreach(_lib ${IL_NAME} ILU ILUT)
  find_library(DevIL_LIBRARY_${_lib}_RELEASE  NAMES ${_lib} HINTS ${DevIL_ROOT})
  find_library(DevIL_LIBRARY_${_lib}_DEBUG  NAMES ${_lib}_d HINTS ${DevIL_ROOT})
  if(DevIL_LIBRARY_${_lib}_RELEASE)
    list(APPEND DevIL_LIBRARIES ${DevIL_LIBRARY_${_lib}_RELEASE})
  endif()

  if(DevIL_LIBRARY_${_lib}_DEBUG)
    list(APPEND DevIL_LIBRARIES ${DevIL_LIBRARY_${_lib}_DEBUG})
  endif()
endforeach()

# Output variables generation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DevIL DEFAULT_MSG DevIL_LIBRARIES
                                                    DevIL_INCLUDE_DIR)

if(DevIL_FOUND)
    if(NOT TARGET DevIL::DevIL)
      add_library(DevIL::DevIL INTERFACE IMPORTED)
    endif()

    foreach(_lib IL ILU ILUT)

      if(NOT TARGET DevIL::${_lib})
        add_library(DevIL::${_lib} UNKNOWN IMPORTED)
        if(EXISTS ${DevIL_LIBRARY_${_lib}_RELEASE})
          set_property(TARGET DevIL::${_lib} APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
          set_target_properties(DevIL::${_lib} PROPERTIES MAP_IMPORTED_CONFIG_RELEASE Release
            IMPORTED_LOCATION_RELEASE "${DevIL_LIBRARY_${_lib}_RELEASE}")
        endif()

        if(EXISTS ${DevIL_LIBRARY_${_lib}_DEBUG})
          set_property(TARGET DevIL::${_lib} APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
          set_target_properties(DevIL::${_lib} PROPERTIES MAP_IMPORTED_CONFIG_DEBUG Debug
            IMPORTED_LOCATION_RELEASE "${DevIL_LIBRARY_${_lib}_DEBUG}")
        endif()

        set_target_properties(DevIL::${_lib} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${DevIL_INCLUDE_DIR})
      endif()

      target_link_libraries(DevIL::DevIL INTERFACE DevIL::${_lib})
    endforeach(_lib IL ILU ILUT)

endif()

mark_as_advanced(DevIL_INCLUDE_DIR
                 DevIL_LIBRARY_RELEASE
                 DevIL_LIBRARY_DEBUG
                 DevIL_LIBRARIES
)

