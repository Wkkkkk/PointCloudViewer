###### headers ######

find_path(LIBSSH_INCLUDE_DIR
        NAMES
        libssh/libssh.h
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

set(LIBSSH_LIB ssh)
set(LIBSSH_LIBRARIES "")
foreach (lib_ ${LIBSSH_LIB})
    FIND_TARGET_LIBRARY(${lib_} ${lib_}_file)
    list(APPEND LIBSSH_LIBRARIES "${${lib_}_file}")
endforeach (lib_)

set(LIBSSH_FOUND FALSE)
IF (LIBSSH_LIBRARIES AND LIBSSH_INCLUDE_DIR)
    set(LIBSSH_FOUND TRUE)
    set(LIBSSH_INCLUDE_DIRS ${LIBSSH_INCLUDE_DIR})

    message(STATUS "find LIBSSH: " ${LIBSSH_INCLUDE_DIRS})
    message(STATUS "contains: " ${LIBSSH_LIBRARIES})
ELSE ()
    message(FATAL_ERROR "can't find LIBSSH")
ENDIF ()