# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_calib_correspondences_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED calib_correspondences_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(calib_correspondences_FOUND FALSE)
  elseif(NOT calib_correspondences_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(calib_correspondences_FOUND FALSE)
  endif()
  return()
endif()
set(_calib_correspondences_CONFIG_INCLUDED TRUE)

# output package information
if(NOT calib_correspondences_FIND_QUIETLY)
  message(STATUS "Found calib_correspondences: 0.0.0 (${calib_correspondences_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'calib_correspondences' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${calib_correspondences_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(calib_correspondences_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${calib_correspondences_DIR}/${_extra}")
endforeach()
