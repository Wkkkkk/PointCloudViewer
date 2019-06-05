/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 5/14/19.
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
#include <memory>

#include <QtCore/QDateTime>
#include <QtCore/QDataStream>

#include <draco/io/ply_decoder.h>
#include <draco/core/decoder_buffer.h>
#include <draco/compression/decode.h>

#include "GPSMsg.h"
#include "Singleton.h"
#include "NetworkReceiver.h"

const int SAMPLE_CNT = 4;
const int MAX_POINT_CNT = 1000;

NetworkReceiver::NetworkReceiver(const QString &name, NetworkReceiver::MODE mode) :
        socket_(new QUdpSocket),
        mode_(mode) {
    this->setObjectName(name);
}

NetworkReceiver::~NetworkReceiver() = default;

void NetworkReceiver::setPortNum(unsigned short port_num) {
    std::cout << this->objectName().toStdString() << " is listening to port: " << port_num << std::endl;
    socket_->bind(QHostAddress::AnyIPv4, port_num);

    if (mode_ == MODE::POINTCLOUD)
        connect(socket_.data(), &QUdpSocket::readyRead, this, &NetworkReceiver::getDracoPointCloudData);
    else if (mode_ == MODE::STATUSINFO)
        connect(socket_.data(), &QUdpSocket::readyRead, this, &NetworkReceiver::getStatusInfo);
    else std::cout << "aha?" << std::endl;
}

void NetworkReceiver::getDracoPointCloudData() {
    QByteArray udp_buffer;
    udp_buffer.resize(socket_->pendingDatagramSize());
    QHostAddress sender;
    quint16 senderPort;
    socket_->readDatagram(udp_buffer.data(), udp_buffer.size(), &sender, &senderPort);

    draco::DecoderBuffer draco_buffer;
    draco_buffer.Init(udp_buffer.data(), udp_buffer.size());
    std::cout << this->objectName().toStdString() << " receive udp buffer size: " << udp_buffer.size() << std::endl;

    std::unique_ptr<draco::PointCloud> draco_point_cloud;
    auto type_statusor = draco::Decoder::GetEncodedGeometryType(&draco_buffer);
    if (!type_statusor.ok()) {
        std::cout << "get a type statusor error, plz check" << std::endl;
        return;
    }
    const draco::EncodedGeometryType geom_type = type_statusor.value();
    if (geom_type == draco::POINT_CLOUD) {
        draco::Decoder decoder;
        auto statusor = decoder.DecodePointCloudFromBuffer(&draco_buffer);
        if (!statusor.ok()) {
            std::cout << "get a decode error" << std::endl;
            return;
        }
        //draco_point_cloud.reset(statusor.value().get());
        draco_point_cloud = std::move(statusor).value();
        if (!draco_point_cloud) {
            std::cout << "Failed to decode the input file" << std::endl;
            return;
        }
	}
	else {
		std::cerr << "get a wrong geometry type: " << geom_type << std::endl;
	}
    auto num_points = draco_point_cloud->num_points();
    std::cout << "convert draco to pcd, point num: " << num_points << std::endl;

    auto pc_att = draco_point_cloud->GetNamedAttribute(draco::GeometryAttribute::POSITION);
	if (!pc_att) {
		std::cerr << "get a wrong geometry attribute" << std::endl;
		return;
	}

    float att_val[3];

    Array points;
    points.reserve(num_points);
    for (draco::AttributeValueIndex i(0); i < pc_att->size(); ++i) {
        pc_att->GetValue(i, att_val);

		Point off_set = Point(att_val[0], att_val[1], att_val[2]);
		if (i.value() % 500 == 0)  std::cout << "point---->offset: " << off_set << std::endl;

        points.push_back(off_set);
    }
	std::cout << "convert to Array, point num: " << points.size() << std::endl;

    //update
    core::Singleton<Array>::getInstance().update(std::move(points));

    //all done
    emit emitPointCloud();
}

void NetworkReceiver::getStatusInfo() {
    QByteArray udp_buffer;
    udp_buffer.resize(socket_->pendingDatagramSize());
    QHostAddress sender;
    quint16 senderPort;
    socket_->readDatagram(udp_buffer.data(), udp_buffer.size(), &sender, &senderPort);
    //std::cout << this->objectName().toStdString() << " get udp buffer size: " << udp_buffer.size() << std::endl;

    GpsMsgs status_info;
    memcpy(&status_info, udp_buffer, sizeof(GpsMsgs));

	//固定点坐标
    Point origin(status_info.ref_x, status_info.ref_y, status_info.ref_z);
	//飞机相对固定点的位移
    Point offset(status_info.x, status_info.y, status_info.z);
	//飞机的绝对坐标
	Point uav_pos = origin + offset;
	
	emit emitRefPos(origin);
    emit emitUAVPos(uav_pos);

    Point gps_loc(status_info.lat, status_info.lon, status_info.hei);
    emit emitGPSLocation(gps_loc);

    QString satellite_num_str = QString::number(status_info.satellites);
    emit emitSatelliteNum(satellite_num_str);

    bool rtk_state = status_info.rtk_state == 50;
    emit emitRTKStatus(rtk_state);

    emit emitGPSLock(status_info.gps_lock);

    emit emitKineticAlignment(status_info.kinetic_alignment);

    emit emitLidarNormal(status_info.lidar_time);
}