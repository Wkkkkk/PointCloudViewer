# This module defines

# OSGEARTH_LIBRARY
# OSGEARTH_FOUND, if false, do not try to link to osg
# OSGEARTH_INCLUDE_DIRS, where to find the headers
# OSGEARTH_INCLUDE_DIR, where to find the source headers

# to use this module, set variables to point to the osg build
# directory, and source directory, respectively
# OSGEARTHDIR or OSGEARTH_SOURCE_DIR: osg source directory, typically OpenSceneGraph
# OSGEARTH_DIR or OSGEARTH_BUILD_DIR: osg build directory, place in which you've
#    built osg via cmake

# Header files are presumed to be included like
# #include <osg/PositionAttitudeTransform>
# #include <osgUtil/SceneView>

###### headers ######

find_path(OSGEARTH_INCLUDE_DIR
        NAMES
        osgEarth/Version
        PATHS
        /usr/include/
        /usr/local/include/
        ${OSGEARTH_DIR}
        $ENV{OSGEARTH_SOURCE_DIR}
        $ENV{OSGEARTHDIR}
        $ENV{OSGEARTH_DIR}
        $ENV{VCPKG_DIR}/installed/x64-windows/include
        )

###### libraries ######
MACRO(FIND_TARGET_LIBRARY MYLIBRARYNAME MYLIBRARY)
    find_library(${MYLIBRARY}
            NAMES
            ${MYLIBRARYNAME}
            PATHS
            /usr/local
            /usr/local/lib
            /usr/local/lib64
            /usr/lib/
            /usr/lib64/
            ${OSGEARTH_DIR}
            $ENV{OSGEARTH_BUILD_DIR}
            $ENV{OSGEARTH_DIR}
            $ENV{OSGEARTHDIR}
            $ENV{OSG_ROOT}
            $ENV{VCPKG_DIR}/installed/x64-windows/lib
            )
ENDMACRO(FIND_TARGET_LIBRARY MYLIBRARYNAME MYLIBRARY)

set(OSGEARTH_LIB osgEarth;osgEarthFeatures;osgEarthUtil;osgEarthAnnotation;osgEarthSymbology)
set(OSGEARTH_LIBRARIES "")
foreach (lib_ ${OSGEARTH_LIB})
    FIND_TARGET_LIBRARY(${lib_} ${lib_}_file)
    list(APPEND OSGEARTH_LIBRARIES "${${lib_}_file}")
endforeach (lib_)

set(OSGEARTH_FOUND FALSE)
IF (OSGEARTH_LIBRARIES AND OSGEARTH_INCLUDE_DIR)
    set(OSGEARTH_FOUND TRUE)
    set(OSGEARTH_INCLUDE_DIRS ${OSGEARTH_INCLUDE_DIR})

    message(STATUS "find osgEarth: " ${OSGEARTH_INCLUDE_DIRS})
    message(STATUS "contains: " ${OSGEARTH_LIBRARIES})
ELSE ()
    message(FATAL_ERROR "can't find osgEarth")
ENDIF ()