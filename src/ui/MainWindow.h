/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 2/15/19.
 * Contact with:wk707060335@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http: *www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/
#ifndef SPACECLOUD_MAINWINDOW_H
#define SPACECLOUD_MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QAction>
#include <QtCore/QScopedPointer>

#include "Common.h"

//forward declaration
class OSGWidget;

class NetworkReceiver;

/**
 * @brief the Mainwindow class
 * It takes control of the whole program
 */
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

    ~MainWindow() final;

    Q_DISABLE_COPY(MainWindow);
private:
    /**
     * @brief It inits all members
     * @param none
     * */
    void initUI();

    void initConfig();
    void createMenu();
    void createToolBar();
    void createDockWidget();

    void createConnect();

    void enableButtons(bool enable);

    //core widget
    OSGWidget*      osgwidget_;

    //other widgets
    QDockWidget*  dock_widget_;
    QTreeWidget*  tree_widget_;
    enum TREE_WIDGET_ITEM {
        GPS_LOCATION,
        SATELLITE_NUM,
        RTK_STATUS,
        GPS_LOCK,
        KINETIC,
        LIDAR
    };

    // basic buttons
    QAction *connect_action_;
    QAction *start_action_;
    QAction *record_action_;
    QAction *end_action_;
    QAction *shutdown_action_;
    QAction *convert_action_;

    // advanced buttons
    QAction *open_file_action_;
    QAction *open_dsm_action_;
    QAction *start_trace_action_;
    QAction *test_trace_action_;

    QScopedPointer<NetworkReceiver> pointcloud_manager_;
    QScopedPointer<NetworkReceiver> statusinfo_manager_;

    std::string ip_address_;
    std::string usr_name_;
    std::string password_;
    std::string connect_cmd_;
    std::string start_cmd_;
    std::string record_cmd_;
    std::string end_cmd_;
    std::string shutdown_cmd_;
signals:
    void test_signal();

private slots:
    void open();

    void updateGPSLocation(Point);

    void updateSatelliteNum(QString);

    void updateRTKStatus(bool);

    void updateGPSLock(bool);

    void updateKineticAligment(bool);

    void updateLidarNormnal(bool);

    void connectTriggered();

    void startTriggered();

    void recordTriggered();

    void endTriggered();

    void shutdownTriggered();

    void convertTriggered();

    void loadFiles();

    void activeTraceRefresh(bool is_active);

    void activeTest(bool is_active);
};

#endif //SPACECLOUD_MAINWINDOW_H
