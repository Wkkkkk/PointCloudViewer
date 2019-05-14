###### headers ######

find_path(DRACO_INCLUDE_DIR
        NAMES
        draco/draco_features.h
        PATHS
        /usr/include/
        /usr/local/include/
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
            $ENV{VCPKG_DIR}/installed/x64-windows/lib
            )
ENDMACRO(FIND_TARGET_LIBRARY MYLIBRARYNAME MYLIBRARY)

set(DRACO_LIB draco dracodec dracoenc)
set(DRACO_LIBRARIES "")
foreach (lib_ ${DRACO_LIB})
    FIND_TARGET_LIBRARY(${lib_} ${lib_}_file)
    list(APPEND DRACO_LIBRARIES "${${lib_}_file}")
endforeach (lib_)

set(DRACO_FOUND FALSE)
IF (DRACO_LIBRARIES AND DRACO_INCLUDE_DIR)
    set(DRACO_FOUND TRUE)
    set(DRACO_INCLUDE_DIRS ${DRACO_INCLUDE_DIR})

    message(STATUS "find DRACO: " ${DRACO_INCLUDE_DIRS})
    message(STATUS "contains: " ${DRACO_LIBRARIES})
ELSE ()
    message(FATAL_ERROR "can't find DRACO")
ENDIF ()