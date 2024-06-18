# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cassie_exp_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cassie_exp_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cassie_exp_FOUND FALSE)
  elseif(NOT cassie_exp_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cassie_exp_FOUND FALSE)
  endif()
  return()
endif()
set(_cassie_exp_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cassie_exp_FIND_QUIETLY)
  message(STATUS "Found cassie_exp: 0.0.0 (${cassie_exp_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cassie_exp' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cassie_exp_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cassie_exp_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${cassie_exp_DIR}/${_extra}")
endforeach()
