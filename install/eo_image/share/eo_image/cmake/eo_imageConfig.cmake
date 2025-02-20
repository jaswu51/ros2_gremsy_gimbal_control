# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_eo_image_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED eo_image_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(eo_image_FOUND FALSE)
  elseif(NOT eo_image_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(eo_image_FOUND FALSE)
  endif()
  return()
endif()
set(_eo_image_CONFIG_INCLUDED TRUE)

# output package information
if(NOT eo_image_FIND_QUIETLY)
  message(STATUS "Found eo_image: 0.0.0 (${eo_image_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'eo_image' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${eo_image_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(eo_image_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${eo_image_DIR}/${_extra}")
endforeach()
