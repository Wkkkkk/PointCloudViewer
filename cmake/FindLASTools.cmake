include(ExternalProject)

ExternalProject_Add(LASTools
        GIT_REPOSITORY https://github.com/LAStools/LAStools.git
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION}
        )
include_directories(${EXTERNAL_INSTALL_LOCATION}/include/LASlib)
link_directories(${EXTERNAL_INSTALL_LOCATION}/lib/LASlib)