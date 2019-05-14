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

#ifndef POINTCLOUDVIEWER_NODETREEINFO_H
#define POINTCLOUDVIEWER_NODETREEINFO_H

#include <iostream>

#include <osg/NodeVisitor>
#include <osg/Switch>
#include <osg/Geode>
#include <osg/Node>
#include <osg/Camera>
#include <osg/PositionAttitudeTransform>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>

namespace osgHelper {
/**
 * @brief helper class
 * print the scene graph
 * a typical use of Visitor Pattern
* */
class NodeTreeInfo : public osg::NodeVisitor {
public:
    NodeTreeInfo() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN),
                     indent_(0) {
        std::cout << "--------------------------------------" << std::endl;
        std::cout << "Print NodeTreeInfo:" << std::endl;
    }

    void apply(osg::Switch &node) override {
        for (int i = 0; i < indent_; i++) std::cout << "  ";
        std::cout << "[" << indent_ + 1 << "]" << node.libraryName() << "::" << node.className() << "::"
                  << node.getName() << std::endl;

        indent_++;
        traverse(node);
        indent_--;

    }

    void apply(osg::Geode &node) override {
        for (int i = 0; i < indent_; i++) std::cout << "  ";
        std::cout << "[" << indent_ + 1 << "]" << node.libraryName() << "::" <<
                  node.className() << "::" << node.getName() << std::endl;

//        for (unsigned int n = 0; n < node.getNumDrawables(); n++){
//        	osg::Drawable* drawable = node.getDrawable(n);
//        	if (!drawable) continue;
//
//        	for (int i = 0; i <= indent_; i++)std::cout << "  ";
//        	std::cout << drawable->libraryName() << "::" << drawable->className() << "::" << drawable->getName() << std::endl;
//        }

        indent_++;
        traverse(node);
        indent_--;
    }

    void apply(osg::Camera &node) override {
        for (int i = 0; i < indent_; i++) std::cout << "  ";
        std::cout << "[" << indent_ + 1 << "]" << node.libraryName() << "::" << node.className() << "::"
                  << node.getName() << std::endl;

        indent_++;
        traverse(node);
        indent_--;
    }

    void apply(osg::PositionAttitudeTransform &node) override {
        for (int i = 0; i < indent_; i++) std::cout << "  ";
        std::cout << "[" << indent_ + 1 << "]" << node.libraryName() << "::" << node.className() << "::"
                  << node.getName() << std::endl;
    }

protected:
    int indent_;
};

/**
* @brief helper class
* interact with user
* a typical use of osgGA::GUIEventHandler
* */
class NodeTreeHandler : public osgGA::GUIEventHandler {
public:
    explicit NodeTreeHandler(osg::Switch *node) : root_node_(node) {}

    /**
     * @brief override the handle() to define action
     * */
    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) final {
        auto myview = dynamic_cast<osgViewer::View *>(&aa);
        if (!myview) return false;

        if (ea.getHandled()) return false;

        switch (ea.getEventType()) {
            case (osgGA::GUIEventAdapter::KEYDOWN) :
                if (ea.getKey() == 'n') {
                    NodeTreeInfo nodeTreeInfo;
                    if (root_node_) root_node_->accept(nodeTreeInfo);
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
}

#endif //POINTCLOUDVIEWER_NODETREEINFO_H
