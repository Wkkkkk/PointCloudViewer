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

#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QProgressDialog>
#include <QtGui/QIcon>

extern "C" {
#include "SSH.h"
}
#include "Config.h"
#include "Singleton.h"
#include "OSGWidget.h"
#include "MainWindow.h"
#include "NetworkReceiver.h"
#include "FileConvertThread.h"

using namespace core;

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        pointcloud_manager_(new NetworkReceiver("PointCloud Receiver", NetworkReceiver::MODE::POINTCLOUD)),
        statusinfo_manager_(new NetworkReceiver("StatusInfo Receiver", NetworkReceiver::MODE::STATUSINFO)) {

    this->setWindowTitle("Lidar receiver");

    osgwidget_ = new OSGWidget(this);
    this->setCentralWidget(osgwidget_);
    osgwidget_->init();

    auto pointcloud_port = Config::get<int>("pointcloud_port", 9696);
    auto statusinfo_port = Config::get<int>("statusinfo_port", 9697);

    pointcloud_manager_->setPortNum(pointcloud_port);
    statusinfo_manager_->setPortNum(statusinfo_port);
    createConnect();

    initUI();
    initConfig();
}

MainWindow::~MainWindow() = default;

void MainWindow::initUI() {
    createMenu();
    createToolBar();
    createDockWidget();
}

void MainWindow::open() {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Model"),
                                                    "/home/zhihui/workspace/data/osg_data", tr("Image Files (*.osg *.osgt *.osgb)"));
    if(fileName.isEmpty()) return;

    QFileInfo f(fileName);
    osgwidget_->readDataFromFile(f);
}

void MainWindow::createMenu() {
    connect_action_ = new QAction(tr("Connect"), this);
    connect_action_->setIcon(QIcon(":/images/connection.png"));
    connect(connect_action_, &QAction::triggered, this, &MainWindow::connectTriggered);

    start_action_ = new QAction(tr("Start"), this);
    start_action_->setIcon(QIcon(":/images/start.png"));
    connect(start_action_, &QAction::triggered, this, &MainWindow::startTriggered);

    record_action_ = new QAction(tr("Record"), this);
    record_action_->setIcon(QIcon(":/images/record.png"));
    connect(record_action_, &QAction::triggered, this, &MainWindow::recordTriggered);

    end_action_ = new QAction(tr("Stop"), this);
    end_action_->setIcon(QIcon(":/images/end.png"));
    connect(end_action_, &QAction::triggered, this, &MainWindow::endTriggered);

    convert_action_ = new QAction(tr("Convert"), this);
    convert_action_->setIcon(QIcon(":/images/convert.png"));
    connect(convert_action_, &QAction::triggered, this, &MainWindow::convertTriggered);

    open_file_action_ = new QAction(tr("Open"), this);
    open_file_action_->setIcon(QIcon(":/images/file_open.png"));
    connect(open_file_action_, &QAction::triggered, this, &MainWindow::open);

    open_dsm_action_ = new QAction(tr("Load"), this);
    open_dsm_action_->setIcon(QIcon(":/images/terrian.png"));
    connect(open_dsm_action_, &QAction::triggered, this, &MainWindow::loadFiles);

    start_trace_action_ = new QAction(tr("Receive"), this);
    start_trace_action_->setIcon(QIcon(":/images/plane.png"));
    start_trace_action_->setCheckable(false);
    connect(open_dsm_action_, &QAction::toggled, this, &MainWindow::activeTraceRefresh);

    test_trace_action_ = new QAction(tr("Test"), this);
    test_trace_action_->setIcon(QIcon(":/images/setup.png"));
    test_trace_action_->setCheckable(true);
    connect(test_trace_action_, &QAction::toggled, this, &MainWindow::activeTest);

    enableButtons(false);
}

void MainWindow::enableButtons(bool enable) {
    start_action_->setEnabled(enable);
    record_action_->setEnabled(enable);
    end_action_->setEnabled(enable);
}

void MainWindow::createToolBar() {
    QToolBar *toolBar = addToolBar("Tools");

    toolBar->addAction(connect_action_);
    toolBar->addAction(start_action_);
    toolBar->addAction(record_action_);
    toolBar->addAction(end_action_);
    toolBar->addAction(convert_action_);
    toolBar->addSeparator();

#ifdef BUILD_FOR_BUILDING_DETECT
    toolBar->addAction(open_dsm_action_);
    toolBar->addAction(start_trace_action_);
    toolBar->addAction(test_trace_action_);
#endif
}

void MainWindow::createDockWidget() {
    tree_widget_ = new QTreeWidget(this);
    tree_widget_->setColumnCount(1);
    tree_widget_->setHeaderHidden(true);
    //tree_widget_->setColumnWidth(0, 100);
    //tree_widget_->setStyleSheet("QTreeWidget::item {height:25px;");

    //GPS位置
    {
        QTreeWidgetItem *item = new QTreeWidgetItem(tree_widget_, QStringList(tr("GPS-location")));
        item->setExpanded(true);
    }

    //卫星数
    {
        QTreeWidgetItem *item = new QTreeWidgetItem(tree_widget_, QStringList(tr("satellite")));
        item->setExpanded(true);
    }

    //RTK状态
    {
        QTreeWidgetItem *item = new QTreeWidgetItem(tree_widget_, QStringList(tr("RTK-status")));
        item->setCheckState(0, Qt::CheckState::Unchecked);
    }

    //GPS lock
    {
        QTreeWidgetItem *item = new QTreeWidgetItem(tree_widget_, QStringList(tr("GPS-lock")));
        item->setCheckState(0, Qt::CheckState::Unchecked);
    }

    //Kinetic Aligment
    {
        QTreeWidgetItem *item = new QTreeWidgetItem(tree_widget_, QStringList(tr("Kinetic")));
        item->setCheckState(0, Qt::CheckState::Unchecked);
    }

    //lidar
    {
        QTreeWidgetItem *item = new QTreeWidgetItem(tree_widget_, QStringList(tr("Lidar")));
        item->setCheckState(0, Qt::CheckState::Unchecked);
    }

    dock_widget_ = new QDockWidget(tr("status"), this);
    dock_widget_->setFixedWidth(200);
    dock_widget_->setFeatures(QDockWidget::AllDockWidgetFeatures);
    dock_widget_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    dock_widget_->setWidget(tree_widget_);
    this->addDockWidget(Qt::LeftDockWidgetArea, dock_widget_);

    //QTreeWidget connect
    //connect(edit_widget_, SIGNAL(itemClicked(QTreeWidgetItem *, int)), this, SLOT(TreeWidgetClicked(QTreeWidgetItem *, int)));
    //connect(edit_widget_, SIGNAL(itemDoubleClicked(QTreeWidgetItem *, int)), this, SLOT(TreeWidgetDoubleClicked(QTreeWidgetItem *, int)));
    //connect(edit_widget_, SIGNAL(itemPressed(QTreeWidgetItem *, int)), this, SLOT(TreeWidgetRightedClicked(QTreeWidgetItem *, int)));
}

void MainWindow::createConnect() {
    connect(statusinfo_manager_.data(), &NetworkReceiver::emitUAVPos, osgwidget_, &OSGWidget::updateUAVPose);
    connect(statusinfo_manager_.data(), &NetworkReceiver::emitGPSLocation, this, &MainWindow::updateGPSLocation);
    connect(statusinfo_manager_.data(), &NetworkReceiver::emitSatelliteNum, this, &MainWindow::updateSatelliteNum);
    connect(statusinfo_manager_.data(), &NetworkReceiver::emitRTKStatus, this, &MainWindow::updateRTKStatus);
    connect(statusinfo_manager_.data(), &NetworkReceiver::emitGPSLock, this, &MainWindow::updateGPSLock);
    connect(statusinfo_manager_.data(), &NetworkReceiver::emitKineticAlignment, this,
            &MainWindow::updateKineticAligment);
    connect(statusinfo_manager_.data(), &NetworkReceiver::emitLidarNormal, this, &MainWindow::updateLidarNormnal);

    connect(pointcloud_manager_.data(), &NetworkReceiver::emitPointCloud, osgwidget_, &OSGWidget::updatePointCloud);
}

void MainWindow::updateGPSLocation(Point p) {
    QString location_str;
    auto location_item = tree_widget_->topLevelItem(GPS_LOCATION);

    if (!location_item->childCount()) {
        auto item = new QTreeWidgetItem(location_item);
    }

    auto item = location_item->child(0);
    item->setText(0, "lat:" + QString::number(p.x(), 'f', 6));
    item->setText(1, "lon:" + QString::number(p.y(), 'f', 6));
    item->setText(2, "hei:" + QString::number(p.z(), 'f', 3));
}

void MainWindow::updateSatelliteNum(QString num) {
    auto satellite_item = tree_widget_->topLevelItem(SATELLITE_NUM);

    if (!satellite_item->childCount()) {
        auto item = new QTreeWidgetItem(satellite_item);
    }

    auto item = satellite_item->child(0);
    item->setText(0, num);
}

void MainWindow::updateRTKStatus(bool is_valid) {
    auto rtk_item = tree_widget_->topLevelItem(RTK_STATUS);

    auto check_state = is_valid ? Qt::CheckState::Checked : Qt::CheckState::Unchecked;
    rtk_item->setCheckState(0, check_state);
}

void MainWindow::updateGPSLock(bool is_valid) {
    auto item = tree_widget_->topLevelItem(GPS_LOCK);

    auto check_state = is_valid ? Qt::CheckState::Checked : Qt::CheckState::Unchecked;
    item->setCheckState(0, check_state);
}

void MainWindow::updateKineticAligment(bool is_valid) {
    auto item = tree_widget_->topLevelItem(KINETIC);

    auto check_state = is_valid ? Qt::CheckState::Checked : Qt::CheckState::Unchecked;
    item->setCheckState(0, check_state);
}

void MainWindow::updateLidarNormnal(bool is_valid) {
    auto item = tree_widget_->topLevelItem(LIDAR);

    auto check_state = is_valid ? Qt::CheckState::Checked : Qt::CheckState::Unchecked;
    item->setCheckState(0, check_state);
}

void MainWindow::loadFiles() {
    auto dsm_file_path = Config::get<std::string>("dsm_file_path", "./data/output/dsm.ive");
    auto building_file_path = Config::get<std::string>("building_file_path", "./data/buildings/indexs.txt");

    osgwidget_->loadDSM(dsm_file_path);
    osgwidget_->loadBuildings(building_file_path);
}

void MainWindow::activeTraceRefresh(bool is_active) {
    osgwidget_->activeTraceRefresh(is_active);
}

void MainWindow::activeTest(bool is_active) {
    osgwidget_->activeTestRefresh(is_active);
}

void MainWindow::initConfig() {
    auto init = [&](std::string &parm, const std::string &name) {
        parm = Config::get<std::string>(name);
    };

    init(ip_address_, "ip_address");
    init(usr_name_, "usr_name");
    init(password_, "password");
    init(connect_cmd_, "connect_bash_file_path");
    init(start_cmd_, "start_bash_file_path");
    init(record_cmd_, "record_bash_file_path");
    init(end_cmd_, "end_bash_file_path");
}

void MainWindow::connectTriggered() {
    int result = execute_ssh_cmd(ip_address_.c_str(), usr_name_.c_str(), password_.c_str(), connect_cmd_.c_str());
    if (result == 0) {
        QMessageBox::information(nullptr, tr("Info"), tr("Connect successfully"), QMessageBox::Yes);

        start_action_->setEnabled(true);
    } else {
        QMessageBox::information(nullptr, tr("Info"), tr("Connect failed"), QMessageBox::Yes);
    }
}

void MainWindow::startTriggered() {
    //非阻塞
    auto start_func = std::bind(&execute_ssh_cmd, ip_address_.c_str(), usr_name_.c_str(), password_.c_str(),
                                start_cmd_.c_str());

    std::thread thread1(start_func);
    thread1.detach();

    start_action_->setEnabled(false);
    record_action_->setEnabled(true);
    end_action_->setEnabled(true);
}

void MainWindow::recordTriggered() {
    //非阻塞
    auto f = std::bind(&execute_ssh_cmd, ip_address_.c_str(), usr_name_.c_str(), password_.c_str(),
                       record_cmd_.c_str());

    std::thread thread(f);
    thread.detach();

    record_action_->setEnabled(false);
}

void MainWindow::endTriggered() {
    int result = execute_ssh_cmd(ip_address_.c_str(), usr_name_.c_str(), password_.c_str(), end_cmd_.c_str());
    if (result == 0) {
        QMessageBox::information(nullptr, tr("Info"), tr("Close successfully"), QMessageBox::Yes);
        start_action_->setEnabled(true);
        record_action_->setEnabled(false);
        end_action_->setEnabled(false);
    }
}

void MainWindow::convertTriggered() {
    QUrl dir_path = QFileDialog::getExistingDirectoryUrl(nullptr, tr("PointCould Import Directory"));
    if (dir_path.isEmpty())
        return;

    auto dlg = new QProgressDialog;
    dlg->setWindowTitle(tr("Convert"));
    dlg->setLabelText(tr("Converting....."));
    dlg->setCancelButton(nullptr);
    dlg->setRange(0, 0);  //always busy

    FileConvertThread *workerThread = new FileConvertThread(dir_path.toLocalFile());
    //connect(workerThread, &fileConvertThread::progress_value, dlg, &QProgressDialog::setValue);
    connect(workerThread, &FileConvertThread::finished, workerThread, &QObject::deleteLater);
    connect(workerThread, &FileConvertThread::finished, dlg, &QProgressDialog::close);
    workerThread->start();

    dlg->exec();
}
