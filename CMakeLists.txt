## =================================================================== ##
## Copyright (C) 2014-2025 Kimura Ryo                                  ##
##                                                                     ##
## This Source Code Form is subject to the terms of the Mozilla Public ##
## License, v. 2.0. If a copy of the MPL was not distributed with this ##
## file, You can obtain one at http://mozilla.org/MPL/2.0/.            ##
## =================================================================== ##

cmake_minimum_required(VERSION 3.31)

# Let AUTOMOC and AUTOUIC process GENERATED files.
cmake_policy(SET CMP0071 NEW)

if(NOT LIBBSDF_DIR)
    set(LIBBSDF_CORE_FOLDER_NAME "libbsdf Core")
    set(LIBBSDF_APPLICATION_FOLDER_NAME "libbsdf Applications")

    set(LIBBSDF_DIR ../libbsdf)
    add_subdirectory(${LIBBSDF_DIR} LibbsdfBin)
endif()
include_directories(${LIBBSDF_DIR}/include)

project(BSDFProcessor)

set(CMAKE_CXX_STANDARD 17)

set(SOURCE_PATH src)

file(GLOB SOURCES
     "${SOURCE_PATH}/*.h"
     "${SOURCE_PATH}/*.cpp"
     "${SOURCE_PATH}/*.c")

include_directories(${SOURCE_PATH})

option(BSDFPROCESSOR_DEVELOPER_MODE "Developer mode" OFF)
if(BSDFPROCESSOR_DEVELOPER_MODE)
    add_definitions(-DDEVELOPER)
endif()

find_package(Eigen3 REQUIRED)

set(OpenSceneGraph_MARK_AS_ADVANCED ON)
find_package(OpenSceneGraph 3.4.0 REQUIRED osgDB osgGA osgText osgUtil osgViewer)
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
find_package(Qt6 COMPONENTS Core Gui Widgets OpenGLWidgets)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

if(MSVC)
    add_compile_options(/W4)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
    add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msse3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3")
endif()

find_package(OpenMP QUIET)
if(OPENMP_FOUND)
    message("OpenMP found")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(RESOURCE_PATH resource)

# Set version of a binary file for Windows
if(MSVC)
    set(RESOURCES ${RESOURCES} ${RESOURCE_PATH}/version.rc)
endif()

# Set icon
if(APPLE)
    set(ICON_FILE ${PROJECT_NAME}.icns)
    set(MACOSX_BUNDLE_ICON_FILE ${ICON_FILE})
    set_source_files_properties(${RESOURCE_PATH}/${ICON_FILE} PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    set(RESOURCES ${RESOURCES} ${RESOURCE_PATH}/${ICON_FILE})
elseif(MSVC)
    set(RESOURCES ${RESOURCES} ${RESOURCE_PATH}/icon.rc)
endif()

set(RESOURCES ${RESOURCES} ${RESOURCE_PATH}/resource.qrc)

add_executable(${PROJECT_NAME} MACOSX_BUNDLE WIN32 ${SOURCES} ${RESOURCES})

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${OPENSCENEGRAPH_LIBRARIES}
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::OpenGLWidgets
    ${OPENGL_LIBRARIES}
    Eigen3::Eigen
    libbsdf)
