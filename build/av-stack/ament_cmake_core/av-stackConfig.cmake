# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_av-stack_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED av-stack_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(av-stack_FOUND FALSE)
  elseif(NOT av-stack_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(av-stack_FOUND FALSE)
  endif()
  return()
endif()
set(_av-stack_CONFIG_INCLUDED TRUE)

# output package information
if(NOT av-stack_FIND_QUIETLY)
  message(STATUS "Found av-stack: 0.0.0 (${av-stack_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'av-stack' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${av-stack_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(av-stack_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${av-stack_DIR}/${_extra}")
endforeach()
