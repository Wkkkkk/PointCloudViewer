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

#ifndef PROTYPE_GPSMSG_H
#define PROTYPE_GPSMSG_H

#include <memory>

#include <Eigen/Dense>

//xia fan
#pragma pack(1)
typedef struct GpsMsgs {
    double x;		//相对于固定点的位移
    double y;
    double z;
    double lat;		//绝对经纬度
    double lon;
    double hei;

    double q_x;
    double q_y;
    double q_z;
    double q_w;
    int satellites;
    int rtk_state;
    int heading_state;
    bool gps_lock;
    bool kinetic_alignment;
    bool lidar_time;
    uint64_t gps_time;

    double ref_x, ref_y, ref_z; //固定点utm坐标(可能为0，0，0)
} GpsMsgs_t;
#pragma pack()

/////////////////////////////////////////////////////

#pragma pack(1)
typedef struct RawIMUFrame {
    double GPSTime;
    int GyroX;
    int GyroY;
    int GyroZ;
    int AccelX;
    int AccelY;
    int AccelZ;
} RawIMUFrame_t;

typedef struct PoseFrame {
    double UTCtime;
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
} PoseFrame_t;

typedef struct PointT {
    uint64_t timestamp;
    float x;
    float y;
    float z;
    uint8_t i;
    uint8_t r;
} PointT_t;
#pragma pack()

/////////////////////////////////////////////////////

typedef struct Meas_ {
    typedef std::shared_ptr<Meas_> Ptr;
    uint64_t timestamp;
    uint32_t seq;
    enum {
        IMU, POSITION, POSS
    };

    virtual int type() const = 0;
} Meas;

typedef struct IMUMeas_ : public Meas_ {
    typedef std::shared_ptr<IMUMeas_> Ptr;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;

    virtual int type() const { return IMU; }
} IMUMeas;

typedef struct POSITIONMeas_ : public Meas_ {
    typedef std::shared_ptr<POSITIONMeas_> Ptr;
    Eigen::Vector3d p;
    Eigen::Matrix3d p_cov;

    virtual int type() const { return POSITION; }
} POSITIONMeas;

typedef struct POSSMeas_ : public Meas_ {
    typedef std::shared_ptr<POSSMeas_> Ptr;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;

    virtual int type() const { return POSS; }
} POSSMeas;

#endif //PROTYPE_GPSMSG_H
