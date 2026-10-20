#/*
# Copyright 2024 Istituto Italiano di Tecnologia (IIT)
# @author Ehsan Ranjbari <ehsan.ranjbari@iit.it>
#* CopyPolicy: Released under the terms of the BSD-3-Clause
#*/

set(ManusGloveDevName "ManusGlove")

if(ENABLE_MANUS_SDK_DOWNLOAD)
    include(DownloadManusSDK)
else()
    find_package(ManusSDK REQUIRED)
endif()

# There is a breaking change from Manus v3.2.1 to v3.2.1 concerning
# an enum type value renaming. So we check the specific header to detect this.
include(CheckCXXSourceCompiles)
get_target_property(MANUS_SDK_INCLUDE_DIRS ManusSDK::ManusSDK INTERFACE_INCLUDE_DIRECTORIES)
set(CMAKE_REQUIRED_INCLUDES ${MANUS_SDK_INCLUDE_DIRS})
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
check_cxx_source_compiles("
#include <ManusSDKTypes.h>
int f() { return ErgonomicsDataType_LeftFingerThumbCMCSpread; }" MANUS_SDK_HAS_THUMB_CMC_ERGONOMICS)
unset(CMAKE_REQUIRED_INCLUDES)
unset(CMAKE_TRY_COMPILE_TARGET_TYPE)

# Export all symbols in Windows
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(YARP_FORCE_DYNAMIC_PLUGINS ON)

# Add a postfix to Windows libraries compiled in debug
if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
    #NOMINMAX is to avoid windows.h defining its own versions of min and max
    add_definitions(-DNOMINMAX)
endif()

# Compile the plugin by default
yarp_prepare_plugin(${ManusGloveDevName} TYPE wearable::devices::${ManusGloveDevName}
                    INCLUDE include/ManusGlove.h
                    CATEGORY device
                    ADVANCED
                    DEFAULT ON
                    )

yarp_add_plugin(${ManusGloveDevName}
    src/ManusGlove.cpp
    src/ManusGloveHelper.cpp
    include/ManusGlove.h
    include/ManusGloveHelper.h
    )

target_include_directories(${ManusGloveDevName} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

if(NOT MANUS_SDK_HAS_THUMB_CMC_ERGONOMICS)
    target_compile_definitions(${ManusGloveDevName} PRIVATE MANUS_VERSION_IS_LESS_THAN_3_2_1)
endif()

target_link_libraries(${ManusGloveDevName}
    PUBLIC
    YARP::YARP_dev
    IWear::IWear
    PRIVATE
    YARP::YARP_init
    ManusSDK::ManusSDK
    Eigen3::Eigen
    )

yarp_install(TARGETS ${ManusGloveDevName}
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
             YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
             )

set (MAIN_XML_FILES conf/ManusGlove.xml)

set (HANDS_XML_FILES conf/ManusGlove/Right.xml
                     conf/ManusGlove/Left.xml)

install(FILES ${MAIN_XML_FILES}
        DESTINATION ${CMAKE_INSTALL_DATADIR}/yarp)

install(FILES ${HANDS_XML_FILES}
        DESTINATION ${CMAKE_INSTALL_DATADIR}/yarp/ManusGlove)
