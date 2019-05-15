set(APP_NAME ${PROJECT_NAME})

file(GLOB SOURCES_CXX *.cpp *.ui *.h *.qrc *.rc)

file(GLOB TRANSLATION_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/tr/lpd_zh.ts)

qt5_add_translation(TRANSLATION_QM ${TRANSLATION_FILES})
add_custom_target(translations
        DEPENDS ${TRANSLATION_QM})

add_custom_target(copy_ts
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${TRANSLATION_QM} ${COPY_OUTPUT_PATH}/tr/lpd_zh.qm
        DEPENDS ${TRANSLATION_QM})

add_custom_target(copy_model
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/model/uav.osgb ${COPY_OUTPUT_PATH}/model/uav.osgb
        )

add_custom_target(copy_fonts
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/fonts/arial.ttf ${COPY_OUTPUT_PATH}/fonts/arial.ttf
        )

add_custom_target(copy_config
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/config/setting.yaml ${COPY_OUTPUT_PATH}/config/setting.yaml
        )

if (WIN32)
    set(PLATFORM WIN32)
endif ()
if (UNIX)
    set(PTHREAD pthread)
endif ()

add_executable(${APP_NAME} ${PLATFORM} ${SOURCES_CXX})
add_dependencies(${APP_NAME} copy_ts copy_model copy_fonts copy_config)

target_link_libraries(${APP_NAME} render core ${OPENSCENEGRAPH_LIBRARIES} ${LIBSSH_LIBRARIES}
        ${PTHREAD} ${Boost_LIBRARIES} LASlib ${LASTools})
qt5_use_modules(${APP_NAME} Sql Gui Widgets OpenGL Network Concurrent)