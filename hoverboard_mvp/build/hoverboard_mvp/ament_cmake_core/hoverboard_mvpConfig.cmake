# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_hoverboard_mvp_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED hoverboard_mvp_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(hoverboard_mvp_FOUND FALSE)
  elseif(NOT hoverboard_mvp_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(hoverboard_mvp_FOUND FALSE)
  endif()
  return()
endif()
set(_hoverboard_mvp_CONFIG_INCLUDED TRUE)

# output package information
if(NOT hoverboard_mvp_FIND_QUIETLY)
  message(STATUS "Found hoverboard_mvp: 0.0.0 (${hoverboard_mvp_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'hoverboard_mvp' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${hoverboard_mvp_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(hoverboard_mvp_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${hoverboard_mvp_DIR}/${_extra}")
endforeach()
