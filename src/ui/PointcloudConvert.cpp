#include <deque>
#include <fstream>
#include <iostream>

#include <lasreader.hpp>
#include <laswriter.hpp>

#include "Config.h"
#include "GPSMsg.h"
#include "PointcloudConvert.h"

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

uint64_t init_time;

std::deque<POSSMeas_::Ptr> poseQue;

Eigen::Isometry3d lidarImuTrans;

Eigen::Vector3d utm_offset;

POSSMeas::Ptr poseInterpolation(uint64_t timestamp) {
    while (poseQue.size() > 1) {
        if ((*(poseQue.begin() + 1))->timestamp > timestamp) {
            break;
        } else {
            poseQue.pop_front();
        }
    }

    if (poseQue.size() > 1 && poseQue.front()->timestamp <= timestamp) {

        double time_duration = (double) ((*(poseQue.begin() + 1))->timestamp - poseQue.front()->timestamp) * 1e-9;
        if (time_duration < 0.05) {
            double t = (double) (timestamp - poseQue.front()->timestamp) * 1e-9 / time_duration;
            POSSMeas::Ptr pose(new POSSMeas);
            pose->p = poseQue.front()->p + t * ((*(poseQue.begin() + 1))->p - poseQue.front()->p);
            pose->q = poseQue.front()->q.slerp(t, (*(poseQue.begin() + 1))->q);
            return pose;
        }
    }
    return nullptr;
}

void readPose(std::string pose_file) {
    poseQue.clear();
    std::ifstream poseFS(pose_file.c_str());
    double timestamp, yaw, pitch, roll;
    while (poseFS.peek() != EOF) {
        std::string line;
        std::getline(poseFS, line);

        // ignore comment lines
        if (line[0] == '#') {
            continue;
        }
        if (!isdigit(line[0])) {
            continue;
        }
        std::stringstream line_stream(line);
        POSSMeas::Ptr outposs(new POSSMeas);
        poseFS >> timestamp >> outposs->p[0] >> outposs->p[1] >> outposs->p[2]
               >> yaw >> pitch >> roll;
        Eigen::AngleAxisd rollAngle(DEG2RAD(pitch), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(DEG2RAD(roll), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(DEG2RAD(-yaw), Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        outposs->timestamp = (uint64_t) (timestamp * 1e9);
        outposs->q = q;
        poseQue.push_back(outposs);
        //        std::cout << outposs->p[0] << outposs->p[1] << outposs->p[2]<< std::endl;
    }
    poseQue.pop_back();
    poseFS.close();
    utm_offset = poseQue.front()->p;
}

Eigen::Vector3d transformPoint(Eigen::Vector3d point, POSSMeas::Ptr pose) {
    Eigen::Isometry3d imuWorldTrans;
    imuWorldTrans = pose->q.toRotationMatrix();
    imuWorldTrans.translation() = pose->p;
    Eigen::Vector3d trans_point;
    trans_point = imuWorldTrans * lidarImuTrans * point;
    return trans_point;
}

void
writeLasFromPointCloud(const char *strInPointsName, const char *strOutLasName, uint64_t time_min, uint64_t time_max,
                       double min_distance, uint64_t lidar_time_offset, const std::vector<int> &id_range) {
	LASreadOpener lasreadopener;
	lasreadopener.set_file_name(strInPointsName);
	if (lasreadopener.active()) {
		LASreader* lasreader = lasreadopener.open();
		if (lasreader == 0) {
			fprintf(stderr, "ERROR: could not open lasreader\n");
			return;
		}
		if (lasreader->header.point_data_format != 1) {
			fprintf(stderr, "ERROR: point format err\n");
			return;
		}
		lasreadopener.get_file_name_number();
		LASwriteOpener laswriteopener;
		laswriteopener.set_file_name(strOutLasName);
		laswriteopener.set_format(LAS_TOOLS_FORMAT_LAZ);
		LASheader lasheader;
		lasheader.x_scale_factor = 0.001;
		lasheader.y_scale_factor = 0.001;
		lasheader.z_scale_factor = 0.001;
		lasheader.x_offset = utm_offset[0];
		lasheader.y_offset = utm_offset[1];
		lasheader.z_offset = utm_offset[2];
		lasheader.point_data_format = 1;
		lasheader.point_data_record_length = 28;

		LASpoint laspoint;
		laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);

		LASwriter* laswriter = laswriteopener.open(&lasheader);
		if (laswriter == 0)
		{
			fprintf(stderr, "ERROR: could not open laswriter\n");
		}
		PointT_t point_t;
		uint64_t time;

		while (lasreader->read_point()) {
			point_t.i = (uint8_t)lasreader->point.get_intensity();
			point_t.r = (uint8_t)lasreader->point.get_point_source_ID();
			point_t.timestamp = (uint64_t)lasreader->point.get_gps_time();
			point_t.x = (float)lasreader->point.get_x();
			point_t.y = (float)lasreader->point.get_y();
			point_t.z = (float)lasreader->point.get_z();
			if (fabs(point_t.x) + fabs(point_t.y) + fabs(point_t.z) < min_distance) {
				continue;
			}

			if (std::find(id_range.begin(), id_range.end(), point_t.r) != id_range.end()) {
				point_t.timestamp += lidar_time_offset;
				if (point_t.timestamp <= time_min || point_t.timestamp >= time_max) {
					if (point_t.timestamp >= time_max) {
						break;
					}
					//std::cout << "point not in period" << std::endl;
					continue;
				}
				Eigen::Vector3d point_lidar(point_t.x, point_t.y, point_t.z);
				POSSMeas::Ptr pose = poseInterpolation(point_t.timestamp);
				if (pose == nullptr) {
					//std::cout << point_t.timestamp << " no pose" << std::endl;
					continue;
				}
				//            std::cout << point_t.timestamp << std::endl;
				if (point_t.timestamp < time) {
					std::cout << "point time err" << std::endl;
				}
				Eigen::Vector3d trans_point;
				trans_point = transformPoint(point_lidar, pose);
				laspoint.set_x(trans_point[0]);
				laspoint.set_y(trans_point[1]);
				laspoint.set_z(trans_point[2]);
				laspoint.set_intensity(point_t.i);
				laspoint.set_point_source_ID(point_t.r);
				laspoint.set_gps_time(point_t.timestamp);
				laswriter->write_point(&laspoint);
				laswriter->update_inventory(&laspoint);
				time = point_t.timestamp;
			}
		}
		laswriter->update_header(&lasheader, TRUE);
		I64 total_bytes = laswriter->close();
		delete laswriter;

	}
}

void ConvertToLas(const std::string &bag_path, std::function<void(void)> callback) {
    std::string /*bag_path,*/ out_file, pose_file, points_file;
    std::string laserTopic;
    std::vector<int> line_id_vec;
    double min_distance;
    double y, p, r;
    double tx, ty, tz;
    double lidar_delay, lidar_period, lidar_time_offset;

    //bag_path = Config::get<std::string>("bag_path");
    out_file = bag_path + Config::get<std::string>("cloud_out", "/home/zhkj/outcloud.ply");
    pose_file = bag_path + Config::get<std::string>("pose_file");
    points_file = bag_path + Config::get<std::string>("points_file");
    line_id_vec = Config::getlist<int>("line_id");
    min_distance = Config::get<double>("min_distance", 1.5);

    y = Config::get<double>("lidar_yaw", 180);
    p = Config::get<double>("lidar_pitch", 0);
    r = Config::get<double>("lidar_roll", 90);
    tx = Config::get<double>("lidar_x", 0.047);
    ty = Config::get<double>("lidar_y", 0.1209);
    tz = Config::get<double>("lidar_z", 0.02571);
    lidar_delay = Config::get<double>("lidar_delay", 30);
    lidar_period = Config::get<double>("lidar_period", 80);
    lidar_time_offset = (uint64_t) (Config::get<double>("lidar_time_offset", 0) * 1e9);

    Eigen::Vector3d ea0(DEG2RAD(y), DEG2RAD(p), DEG2RAD(r)); //yaw,pitching,roll
//    Eigen::Vector3d ea0(DEG2RAD(-180), DEG2RAD(15), DEG2RAD(-180)); //yaw,pitching,roll
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(ea0[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ());
    lidarImuTrans = R;
    lidarImuTrans.translation() = Eigen::Vector3d(tx, ty, tz);

    for (uint8_t i = 0; i < 0; i++) {
        readPose(pose_file);
        uint64_t time_min = poseQue.front()->timestamp + (uint64_t) (lidar_delay * 1e9);
        uint64_t time_max = time_min + (uint64_t) (lidar_period * 1e9);
        std::string file = out_file + "_" + std::to_string(i);
        std::vector<int> id_range = {i};
        writeLasFromPointCloud(points_file.c_str(), file.c_str(), time_min, time_max, min_distance, lidar_time_offset,
                               id_range);
    }

    readPose(pose_file);
    uint64_t time_min = poseQue.front()->timestamp + (uint64_t) (lidar_delay * 1e9);
    uint64_t time_max = time_min + (uint64_t) (lidar_period * 1e9);
    writeLasFromPointCloud(points_file.c_str(), out_file.c_str(), time_min, time_max, min_distance, lidar_time_offset,
                           line_id_vec);
}
