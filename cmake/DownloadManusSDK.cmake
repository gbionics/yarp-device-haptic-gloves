# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

include(FetchContent)

# Configuration for Manus SDK download
set(MANUS_SDK_DOWNLOAD_VERSION "3.2.1" CACHE STRING "Manus SDK version to download")

if(MANUS_SDK_DOWNLOAD_VERSION VERSION_LESS "3.2.1")
  set(_manus_sdk_archive_name "MANUS_Core_${MANUS_SDK_DOWNLOAD_VERSION}_SDK.zip")
  set(_manus_sdk_hash_key "${MANUS_SDK_DOWNLOAD_VERSION}")
elseif(WIN32)
  set(_manus_sdk_archive_name "MANUS_Core_${MANUS_SDK_DOWNLOAD_VERSION}_SDK_Windows.zip")
  set(_manus_sdk_hash_key "${MANUS_SDK_DOWNLOAD_VERSION}-Windows")
else()
  set(_manus_sdk_archive_name "MANUS_Core_${MANUS_SDK_DOWNLOAD_VERSION}_SDK_Linux.tar.gz")
  set(_manus_sdk_hash_key "${MANUS_SDK_DOWNLOAD_VERSION}-Linux")
endif()

# Version-specific SHA256 hashes for integrity verification
# Note: Update these hashes when adding support for new SDK versions
set(_manus_sdk_version_hashes
    "2.5.1=SHA256=7384c08e493ea2077fe7038701f47881f05c4b37e7f9c621fc0fcf285718da72"
    "3.1.1=SHA256=c5ccd3c42a501107ec79f70d8450a486fbc3925c5c1e18e606114d09f2d9d24a"
    "3.2.1-Linux=SHA256=c2bafd19824ecb458d34e2f559fbd51dd1c7118288c75c24ec5163d6b4135f46"
    "3.2.1-Windows=SHA256=c8f6ffc2c2c477150c1921229dce6a495b3972d612d23efba4b69813aa33d7f0"
)

set(_manus_sdk_url_hash "")
foreach(_hash_entry ${_manus_sdk_version_hashes})
    string(REGEX MATCH "^([^=]+)=(.+)$" _hash_matched "${_hash_entry}")
    if(CMAKE_MATCH_1 STREQUAL _manus_sdk_hash_key)
        set(_manus_sdk_url_hash "${CMAKE_MATCH_2}")
        break()
    endif()
endforeach()

if(NOT _manus_sdk_url_hash)
  message(FATAL_ERROR 
    "ManusSDK version ${MANUS_SDK_DOWNLOAD_VERSION} is not supported.\n"
    "Supported versions are:\n"
    "  - 2.5.1\n"
    "  - 3.1.1\n"
    "  - 3.2.1\n"
    "Please set MANUS_SDK_DOWNLOAD_VERSION to one of the supported versions.")
endif()

if(MANUS_SDK_DOWNLOAD_VERSION VERSION_LESS "3.0.0")
  set(_manus_sdk_major 2)
else()
  set(_manus_sdk_major 3)
endif()
set(_manus_sdk_url "https://static.manus-meta.com/resources/manus_core_${_manus_sdk_major}/sdk/${_manus_sdk_archive_name}")

# Fetch ManusSDK from the official Manus static server
FetchContent_Declare(
    ManusSDK
    URL ${_manus_sdk_url}
    URL_HASH ${_manus_sdk_url_hash}
    SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/manussdk-src
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_MakeAvailable(ManusSDK)

# Determine the SDK client directory based on platform
set(_MANUS_SDK_EXTRACT_ROOT ${CMAKE_BINARY_DIR}/_deps/manussdk-src)

# Debug: List the contents of the extracted directory
if(NOT EXISTS "${_MANUS_SDK_EXTRACT_ROOT}")
  message(FATAL_ERROR "ManusSDK extraction directory not found: ${_MANUS_SDK_EXTRACT_ROOT}")
endif()

file(GLOB _manus_sdk_top_level RELATIVE "${_MANUS_SDK_EXTRACT_ROOT}" "${_MANUS_SDK_EXTRACT_ROOT}/*")
message(STATUS "ManusSDK extracted contents: ${_manus_sdk_top_level}")

# Handle nested SDKClient.zip for older SDK versions (< 3.0.0)
set(_manus_sdk_nested_zip "${_MANUS_SDK_EXTRACT_ROOT}/SDKClient.zip")
if(EXISTS "${_manus_sdk_nested_zip}")
  message(STATUS "Extracting nested SDKClient.zip")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xf "${_manus_sdk_nested_zip}"
    WORKING_DIRECTORY "${_MANUS_SDK_EXTRACT_ROOT}"
    RESULT_VARIABLE _manus_sdk_nested_extract_result
  )
  if(NOT _manus_sdk_nested_extract_result EQUAL 0)
    message(FATAL_ERROR "Failed to extract nested SDKClient.zip")
  endif()
endif()

# The archive structure does not have a versioned subdirectory, SDKClient_* are at the root
if(MANUS_SDK_DOWNLOAD_VERSION VERSION_GREATER_EQUAL "3.2.1")
  set(_manus_sdk_client_dir "${_MANUS_SDK_EXTRACT_ROOT}/C++/SDKClient")
elseif(UNIX)
  set(_manus_sdk_client_dir "${_MANUS_SDK_EXTRACT_ROOT}/SDKClient_Linux")
else()
  set(_manus_sdk_client_dir "${_MANUS_SDK_EXTRACT_ROOT}/SDKClient_Windows")
endif()

# Verify the client directory exists
if(NOT EXISTS "${_manus_sdk_client_dir}")
  message(FATAL_ERROR "ManusSDK client directory not found at: ${_manus_sdk_client_dir}\nPlease verify the SDK version and archive structure.")
endif()

# Create an imported target for ManusSDK since it's a prebuilt binary SDK
add_library(ManusSDK::ManusSDK SHARED IMPORTED GLOBAL)

if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(_MANUS_SDK_ARCH x64)
    else()
        set(_MANUS_SDK_ARCH x86)
    endif()
    
    set(_MANUS_SDK_DLL "${_manus_sdk_client_dir}/ManusSDK/lib/ManusSDK.dll")
    set(_MANUS_SDK_LIB "${_manus_sdk_client_dir}/ManusSDK/lib/ManusSDK.lib")
    set(_MANUS_SDK_INCLUDE "${_manus_sdk_client_dir}/ManusSDK/include")
    
    # Verify the files exist
    if(NOT EXISTS "${_MANUS_SDK_DLL}")
        message(FATAL_ERROR "ManusSDK DLL not found at: ${_MANUS_SDK_DLL}")
    endif()
    if(NOT EXISTS "${_MANUS_SDK_LIB}")
        message(FATAL_ERROR "ManusSDK LIB not found at: ${_MANUS_SDK_LIB}")
    endif()
    if(NOT EXISTS "${_MANUS_SDK_INCLUDE}")
        message(FATAL_ERROR "ManusSDK include directory not found at: ${_MANUS_SDK_INCLUDE}")
    endif()
    
    set_target_properties(ManusSDK::ManusSDK PROPERTIES
        IMPORTED_LOCATION "${_MANUS_SDK_DLL}"
        IMPORTED_IMPLIB "${_MANUS_SDK_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${_MANUS_SDK_INCLUDE}"
        IMPORTED_LOCATION_RELEASE "${_MANUS_SDK_DLL}"
        IMPORTED_IMPLIB_RELEASE "${_MANUS_SDK_LIB}"
    )
    
    # For multi-config generators, also set Release-specific properties
    set_target_properties(ManusSDK::ManusSDK PROPERTIES
        MAP_IMPORTED_CONFIG_RELEASE Release
        MAP_IMPORTED_CONFIG_DEBUG Release
    )
    
elseif(APPLE)
    set(_MANUS_SDK_LIB "${_manus_sdk_client_dir}/ManusSDK/lib/libManusSDK.dylib")
    set(_MANUS_SDK_INCLUDE "${_manus_sdk_client_dir}/ManusSDK/include")
    
    if(NOT EXISTS "${_MANUS_SDK_LIB}")
        message(FATAL_ERROR "ManusSDK dylib not found at: ${_MANUS_SDK_LIB}")
    endif()
    if(NOT EXISTS "${_MANUS_SDK_INCLUDE}")
        message(FATAL_ERROR "ManusSDK include directory not found at: ${_MANUS_SDK_INCLUDE}")
    endif()
    
    set_target_properties(ManusSDK::ManusSDK PROPERTIES
        IMPORTED_LOCATION "${_MANUS_SDK_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${_MANUS_SDK_INCLUDE}"
        IMPORTED_LOCATION_RELEASE "${_MANUS_SDK_LIB}"
    )
    
elseif(UNIX)
    # On Linux, link against the "Integrated" variant. The plain libManusSDK.so
    # dynamically depends on libgrpc.so.9, libgrpc++.so.1, libprotobuf.so.22 and
    # libzmq.so.5, which are neither shipped in the SDK nor available as system
    # packages, causing a runtime "cannot open shared object file" failure.
    # libManusSDK_Integrated.so statically bundles those dependencies and only
    # needs standard system libraries.
    if(MANUS_SDK_DOWNLOAD_VERSION VERSION_GREATER_EQUAL "3.2.1")
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
            set(_MANUS_SDK_LINUX_ARCH aarch64)
        else()
            set(_MANUS_SDK_LINUX_ARCH amd64)
        endif()
        set(_MANUS_SDK_LIB "${_manus_sdk_client_dir}/ManusSDK/lib/${_MANUS_SDK_LINUX_ARCH}/libManusSDK-${_MANUS_SDK_LINUX_ARCH}.so")
    else()
        set(_MANUS_SDK_LIB "${_manus_sdk_client_dir}/ManusSDK/lib/libManusSDK_Integrated.so")
    endif()
    set(_MANUS_SDK_INCLUDE "${_manus_sdk_client_dir}/ManusSDK/include")
    
    if(NOT EXISTS "${_MANUS_SDK_LIB}")
        message(FATAL_ERROR "ManusSDK shared library not found at: ${_MANUS_SDK_LIB}")
    endif()
    if(NOT EXISTS "${_MANUS_SDK_INCLUDE}")
        message(FATAL_ERROR "ManusSDK include directory not found at: ${_MANUS_SDK_INCLUDE}")
    endif()

    # The prebuilt Manus SDK shared library ships without a DT_SONAME. By
    # default CMake links imported libraries by full path, which makes the
    # linker bake that (build-relative) path into the consumer's DT_NEEDED
    # entry, breaking it at runtime. IMPORTED_NO_SONAME makes CMake link via
    # -L<dir> -l<name> so the linker records only the basename
    # (libManusSDK_Integrated.so), resolved at runtime through the
    # consumer's RPATH ($ORIGIN/../lib). Note: this property is only honored
    # for targets CMake treats as shared libraries, hence SHARED IMPORTED
    # (not UNKNOWN IMPORTED).
    set_target_properties(ManusSDK::ManusSDK PROPERTIES
        IMPORTED_LOCATION "${_MANUS_SDK_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${_MANUS_SDK_INCLUDE}"
        IMPORTED_LOCATION_RELEASE "${_MANUS_SDK_LIB}"
        IMPORTED_NO_SONAME TRUE
    )
endif()

# Store the SDK root for later use in installation
set(MANUS_SDK_ROOT "${_manus_sdk_client_dir}" CACHE INTERNAL "ManusSDK root directory")

# Install the ManusSDK library
if(WIN32)
    # On Windows, install the DLL to bin directory and the import library to lib directory
    get_target_property(_manus_sdk_dll_location ManusSDK::ManusSDK IMPORTED_LOCATION)
    if(_manus_sdk_dll_location)
        install(FILES "${_manus_sdk_dll_location}"
                DESTINATION ${CMAKE_INSTALL_BINDIR}
                COMPONENT runtime)
    endif()
    
    get_target_property(_manus_sdk_implib_location ManusSDK::ManusSDK IMPORTED_IMPLIB)
    if(_manus_sdk_implib_location)
        install(FILES "${_manus_sdk_implib_location}"
                DESTINATION ${CMAKE_INSTALL_LIBDIR}
                COMPONENT runtime)
    endif()
else()
    # On Unix-like systems, install the shared library to lib directory
    get_target_property(_manus_sdk_lib_location ManusSDK::ManusSDK IMPORTED_LOCATION)
    if(_manus_sdk_lib_location)
        install(FILES "${_manus_sdk_lib_location}"
                DESTINATION ${CMAKE_INSTALL_LIBDIR}
                COMPONENT runtime)
    endif()
endif()

message(STATUS "Downloaded Manus SDK ${MANUS_SDK_DOWNLOAD_VERSION} from ${_manus_sdk_url}")
message(STATUS "Using Manus SDK ${MANUS_SDK_DOWNLOAD_VERSION} from ${MANUS_SDK_ROOT}")
