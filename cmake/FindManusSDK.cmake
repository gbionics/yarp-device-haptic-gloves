# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

include(FindPackageHandleStandardArgs)

set(MANUS_ROOT_DIR "$ENV{ManusGlove_DIR}" CACHE PATH "Folder containing the ManusSDK")

find_path(MANUS_INCLUDE_DIR ManusSDK.h PATHS ${MANUS_ROOT_DIR} PATH_SUFFIXES ManusSDK/include)
if(UNIX)
  set(_manus_lib_name ManusSDK_integrated)
else()
  set(_manus_lib_name ManusSDK)
endif()
find_library(MANUS_LIBRARY ${_manus_lib_name} PATHS ${MANUS_ROOT_DIR} PATH_SUFFIXES ManusSDK ManusSDK/lib)

find_package_handle_standard_args(ManusSDK DEFAULT_MSG MANUS_INCLUDE_DIR MANUS_LIBRARY)

if(ManusSDK_FOUND)
    if(NOT TARGET ManusSDK::ManusSDK)
      add_library(ManusSDK::ManusSDK UNKNOWN IMPORTED)
      set_target_properties(ManusSDK::ManusSDK PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${MANUS_INCLUDE_DIR}")
      set_property(TARGET ManusSDK::ManusSDK APPEND PROPERTY
        IMPORTED_LOCATION "${MANUS_LIBRARY}")
    endif()
endif()
