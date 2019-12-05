## =================================================================== ##
## Copyright (C) 2014-2019 Kimura Ryo                                  ##
##                                                                     ##
## This Source Code Form is subject to the terms of the Mozilla Public ##
## License, v. 2.0. If a copy of the MPL was not distributed with this ##
## file, You can obtain one at http://mozilla.org/MPL/2.0/.            ##
## =================================================================== ##

cmake_minimum_required(VERSION 3.1.0)

# Let AUTOMOC and AUTOUIC process GENERATED files.
cmake_policy(SET CMP0071 NEW)

# Enable double precision vector types
add_definitions(-DLIBBSDF_DOUBLE_PRECISION_VECTOR)

if(NOT LIBBSDF_DIR)
    set(LIBBSDF_CORE_FOLDER_NAME "libbsdf Core")
    set(LIBBSDF_APPLICATION_FOLDER_NAME "libbsdf Applications")

    set(LIBBSDF_DIR ../libbsdf)
    add_subdirectory(${LIBBSDF_DIR} LibbsdfBin)
endif()
include_directories(${LIBBSDF_DIR}/include)

project(BSDFProcessor)

set(SOURCE_PATH src)

file(GLOB SOURCES
     "${SOURCE_PATH}/*.h"
     "${SOURCE_PATH}/*.cpp"
     "${SOURCE_PATH}/*.c")

include_directories(${SOURCE_PATH})

# Enable all warnings
#add_definitions(-Wall)

if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Please set the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
include_directories("${EIGEN3_INCLUDE_DIR}")

set(OpenSceneGraph_MARK_AS_ADVANCED ON)
find_package(OpenSceneGraph 3.4.0 REQUIRED osgDB osgGA osgText osgUtil osgViewer)
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
find_package(Qt5 COMPONENTS Widgets OpenGL REQUIRED)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")

    # 32 bit executable
    if(CMAKE_SIZEOF_VOID_P MATCHES 4)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:SSE2")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:SSE2")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LARGEADDRESSAWARE")
    endif()
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

if(APPLE)
    set(ICON_FILE ${PROJECT_NAME}.icns)
    set(MACOSX_BUNDLE_ICON_FILE ${ICON_FILE})
    set_source_files_properties(icon/${ICON_FILE} PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    set(SOURCES ${SOURCES} icon/${ICON_FILE})
elseif(MSVC)
    set(SOURCES ${SOURCES} icon/${PROJECT_NAME}.rc)
endif()

qt5_add_resources(RESOURCE resource/resource.qrc)

add_executable(${PROJECT_NAME} MACOSX_BUNDLE WIN32 ${SOURCES} ${RESOURCE})

target_link_libraries(${PROJECT_NAME}
                      ${OPENSCENEGRAPH_LIBRARIES} Qt5::Widgets Qt5::OpenGL ${OPENGL_LIBRARIES}
                      libbsdf)
