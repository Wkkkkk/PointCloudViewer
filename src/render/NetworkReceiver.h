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

#ifndef PROTYPE_NETWORKRECEIVER_H
#define PROTYPE_NETWORKRECEIVER_H

#include <QtCore/QObject>
#include <QtNetwork/QUdpSocket>

#include "Common.h"

class NetworkReceiver : public QObject {
Q_OBJECT
public:
    enum class MODE {
        POINTCLOUD,
        STATUSINFO
    };

    explicit NetworkReceiver(const QString &name, MODE mode);

    ~NetworkReceiver() final;

    void setPortNum(unsigned short port_num);

    Q_DISABLE_COPY(NetworkReceiver);
private:
    QScopedPointer<QUdpSocket> socket_;
    const MODE mode_;

Q_SIGNALS:

    void emitPointCloud();

	void emitRefPos(Point);

    void emitUAVPos(Point);

    void emitGPSLocation(Point);

    void emitSatelliteNum(QString);

    void emitRTKStatus(bool);

    void emitGPSLock(bool);

    void emitKineticAlignment(bool);

    void emitLidarNormal(bool);

private Q_SLOTS:

    void getDracoPointCloudData();

    void getStatusInfo();
};

#endif //PROTYPE_NETWORKRECEIVER_H
