# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_gimbal_zoom_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED gimbal_zoom_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(gimbal_zoom_FOUND FALSE)
  elseif(NOT gimbal_zoom_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(gimbal_zoom_FOUND FALSE)
  endif()
  return()
endif()
set(_gimbal_zoom_CONFIG_INCLUDED TRUE)

# output package information
if(NOT gimbal_zoom_FIND_QUIETLY)
  message(STATUS "Found gimbal_zoom: 0.0.0 (${gimbal_zoom_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'gimbal_zoom' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${gimbal_zoom_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(gimbal_zoom_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${gimbal_zoom_DIR}/${_extra}")
endforeach()
