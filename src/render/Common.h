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

#ifndef SPACECLOUD_COMMON_H
#define SPACECLOUD_COMMON_H

#include <vector>
#include <memory>
#include <iostream>

#include <osg/Vec3d>

typedef osg::Vec3d Point;
typedef std::vector<osg::Vec3d> Array;

struct Result {
    int plane_id;
    int build_id;
    Point center;
    Point plane_llh;
    double build_area;
    double change_rate;
    bool is_illegal;
    std::string video_url;

    Result(int i, int id, Point c, Point llh, double b_a, double c_a, bool legal, std::string url) :
            plane_id(i), build_id(id), center(c), plane_llh(llh),
            build_area(b_a), change_rate(c_a), is_illegal(legal), video_url(std::move(url)) {}
};

const char root_node_name[] = "root_node";
const char point_cloud_node_name[] = "point_cloud_node";
const char uav_node_name[] = "uav_node";
const char ori_node_name[] = "ori_node";
const char hud_node_name[] = "hud_node";
const char text_node_name[] = "text_node";
const char positon_geode_name[] = "positon_geode";
const char helper_node_name[] = "helper_node";

const char dsm_node_name[] = "dsm_node";
const char building_node_name[] = "building_node";
const char model_node_name[] = "model_node";
const char building_trans_node_name[] = "building_trans_node";
const char plane_trans_node_name[] = "plane_trans_node";

inline std::ostream &operator<<( std::ostream &output, const osg::Vec3d &p )
{
    output << p.x() << " " << p.y() << " " << p.z();
    return output;
}
#endif //SPACECLOUD_COMMON_H
