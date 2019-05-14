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
#include <QtGui/QIcon>

#include "Config.h"
#include "Singleton.h"
#include "OSGWidget.h"
#include "MainWindow.h"
#include "NetworkReceiver.h"

using namespace core;

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        pointcloud_manager_(new NetworkReceiver("PointCloud Receiver", NetworkReceiver::MODE::POINTCLOUD)),
        statusinfo_manager_(new NetworkReceiver("StatusInfo Receiver", NetworkReceiver::MODE::STATUSINFO)) {

    this->setWindowTitle("Protype");

    osgwidget_ = new OSGWidget(this);
    this->setCentralWidget(osgwidget_);
    osgwidget_->init();

    auto pointcloud_port = Config::get<int>("pointcloud_port", 9696);
    auto statusinfo_port = Config::get<int>("statusinfo_port", 9697);

    pointcloud_manager_->setPortNum(pointcloud_port);
    statusinfo_manager_->setPortNum(statusinfo_port);
    createConnect();

    initUI();
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
    open_file_action_ = new QAction(tr("Open"), this);
    open_file_action_->setIcon(QIcon(":/images/file_open.png"));
    connect(open_file_action_, &QAction::triggered, this, &MainWindow::open);
}

void MainWindow::createToolBar() {
    QToolBar *toolBar = addToolBar("Tools");

    toolBar->addAction(open_file_action_);
    toolBar->addSeparator();
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
        QTreeWidgetItem *item = new QTreeWidgetItem(tree_widget_, QStringList(tr("statelite")));
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