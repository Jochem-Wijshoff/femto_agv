# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_femto_agv_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED femto_agv_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(femto_agv_FOUND FALSE)
  elseif(NOT femto_agv_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(femto_agv_FOUND FALSE)
  endif()
  return()
endif()
set(_femto_agv_CONFIG_INCLUDED TRUE)

# output package information
if(NOT femto_agv_FIND_QUIETLY)
  message(STATUS "Found femto_agv: 0.0.0 (${femto_agv_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'femto_agv' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT femto_agv_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(femto_agv_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${femto_agv_DIR}/${_extra}")
endforeach()
