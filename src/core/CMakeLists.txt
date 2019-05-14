get_filename_component(DIR_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(MODULE_NAME ${DIR_NAME})

FILE(GLOB ${MODULE_NAME}_SRC *.cpp *.ui *.h *.qrc)
add_library(${MODULE_NAME} ${${MODULE_NAME}_SRC})
target_link_libraries(${MODULE_NAME} yaml-cpp)