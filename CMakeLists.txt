cmake_minimum_required(VERSION 3.5)
project(protype)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
set(COPY_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH})
set(EXTERNAL_INSTALL_LOCATION ${PROJECT_SOURCE_DIR}/external)
set(CMAKE_CXX_FLAGS "-DBOOST_ALL_DYN_LINK")
add_definitions(-D_CRT_SECURE_NO_WARNINGS)
add_definitions(-DBUILD_FOR_BUILDING_DETECT)

#output setting for windows compatibility
IF (WIN32)
    set(COPY_OUTPUT_PATH ${COPY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWIN32")  # for #include<gl/gl.h> bugs
    add_definitions("/EHsc") # boost::throw_exception bugs
ENDIF ()

#find 3rdparty
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Qt5LinguistTools)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Gui CONFIG REQUIRED)
find_package(Qt5Network CONFIG REQUIRED)
find_package(OpenSceneGraph REQUIRED COMPONENTS osgGA osgViewer osgDB osgUtil osgText osg osgFX)
find_package(Boost COMPONENTS log thread date_time filesystem system REQUIRED)
find_package(Draco REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(libssh REQUIRED)

message(STATUS "libssh:${LIBSSH_LIBRARIES}")

include_directories(
        ${Boost_INCLUDE_DIR}
        ${OPENSCENEGRAPH_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        thirdparty/LAStools/LASlib/inc
        thirdparty/LAStools/LASzip/src
)

link_directories(
    ${Boost_LIBRARY_DIR}
    ${OPENSCENEGRAPH_LIBRARY_DIRS}
)

add_subdirectory(src)
add_subdirectory(thirdparty/LAStools)