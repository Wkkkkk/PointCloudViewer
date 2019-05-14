###### headers ######

find_path(OPENSCENEGRAPH_INCLUDE_DIR
        NAMES
        osg/Version
        PATHS
        /usr/include/
        /usr/local/include/
        ${OSG_DIR}
        $ENV{OSG_SOURCE_DIR}
        $ENV{OSGDIR}
        $ENV{OSG_DIR}
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
            /usr/lib/x86_64-linux-gnu
            /usr/lib64/
            ${OSG_DIR}
            $ENV{OSG_DIR}
            $ENV{OSGIDR}
            $ENV{VCPKG_DIR}/installed/x64-windows/lib
            )
ENDMACRO(FIND_TARGET_LIBRARY MYLIBRARYNAME MYLIBRARY)


set(OPENSCENEGRAPH_LIB osg osgAnimation osgDB osgFX osgGA osgManipulator osgParticle osgPresentation osgShadow
        osgSim osgTerrain osgText osgUI osgUtil osgViewer osgVolume osgWidget OpenThreads)
set(OPENSCENEGRAPH_LIBRARIES "")
foreach (lib_ ${OPENSCENEGRAPH_LIB})
    FIND_TARGET_LIBRARY(${lib_} ${lib_}_file)
    list(APPEND OPENSCENEGRAPH_LIBRARIES "${${lib_}_file}")
endforeach (lib_)

set(OPENSCENEGRAPH_FOUND FALSE)
IF (OPENSCENEGRAPH_LIBRARIES AND OPENSCENEGRAPH_INCLUDE_DIR)
    set(OPENSCENEGRAPH_FOUND TRUE)
    set(OPENSCENEGRAPH_INCLUDE_DIRS ${OPENSCENEGRAPH_INCLUDE_DIR})

    message(STATUS "find osg: " ${OPENSCENEGRAPH_INCLUDE_DIR})
    message(STATUS "contains: " ${OPENSCENEGRAPH_LIBRARIES})
ELSE ()
    message(FATAL_ERROR "can't find openscenegraph")
    message(FATAL_ERROR "OPENSCENEGRAPH_INCLUDE_DIR:${OPENSCENEGRAPH_INCLUDE_DIR}")
    message(FATAL_ERROR "OPENSCENEGRAPH_LIBRARIES:${OPENSCENEGRAPH_LIBRARIES}")
ENDIF ()