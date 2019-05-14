/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 11/19/18.
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

#ifndef POINTCLOUDVIEWER_NODECALLBACK_H
#define POINTCLOUDVIEWER_NODECALLBACK_H

#include <osg/Math>
#include <osg/Vec3d>
#include <osg/NodeCallback>

namespace osgHelper {
/**
 * @brief helper class
 * an example of osg::Nodecallback to
 * show how to update a node
 * */
class NodeCallback : public osg::NodeCallback {
public:
    NodeCallback();

    ~NodeCallback() final;

    void operator()(osg::Node *node, osg::NodeVisitor *nv) final;

private:
    double angle_;
    const double radius_;
    const osg::Vec3d center_;
};
}

#endif //POINTCLOUDVIEWER_NODECALLBACK_H
