/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 1/7/19.
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

#ifndef POINTCLOUDVIEWER_CLEANHANDLER_H
#define POINTCLOUDVIEWER_CLEANHANDLER_H

#include <osg/NodeVisitor>
#include <osg/Switch>
#include <osg/Geode>
#include <osg/Node>
#include <osg/Camera>
#include <osg/PositionAttitudeTransform>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>

#include "Common.h"
#include "NodeTreeSearch.h"

class CleanHandler : public osgGA::GUIEventHandler {
public:
    explicit CleanHandler(osg::Switch* node):root_node_(node){}

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override
    {
        auto myview = dynamic_cast<osgViewer::View*>(&aa);
        if (!myview) return false;

        if (ea.getHandled()) return false;

        switch (ea.getEventType()) {
            case(osgGA::GUIEventAdapter::KEYDOWN) :
                if (ea.getKey() == osgGA::GUIEventAdapter::KEY_C)
                {
                    static osg::ref_ptr<osg::Switch> point_cloud_node = dynamic_cast<osg::Switch*>(
                            osgHelper::NodeTreeSearch::findNodeWithName(root_node_, point_cloud_node_name));

                    if(point_cloud_node.valid())
                    {
                        point_cloud_node->removeChildren(0, point_cloud_node->getNumChildren());
                        std::cout << "remove all point cloud..." << std::endl;
                    }

                    return true;
                }
            default:
                break;
        }

        return false;

    }

protected:
    osg::ref_ptr<osg::Switch> root_node_;
};


#endif //POINTCLOUDVIEWER_CLEANHANDLER_H
