#ifndef POINTCLOUDVIEWER_NODETREESEARCH_H
#define POINTCLOUDVIEWER_NODETREESEARCH_H

#include <string>
#include <iostream>

#include <osg/Node>
#include <osg/Geode>
#include <osg/Switch>
#include <osg/PositionAttitudeTransform>
#include <osg/NodeVisitor>

namespace osgHelper {
/**
 * @brief helper class
 * search the node with given name in scene graph
 * a typical use of Visitor Pattern
* */
class NodeTreeSearch : public osg::NodeVisitor {
public:
    explicit NodeTreeSearch(const char name[]) :
            osg::NodeVisitor(TRAVERSE_ALL_CHILDREN),
            node_(nullptr),
            name_(name) {
    }

    void apply(osg::Switch &search_node) override {
        if (search_node.getName() == name_) {
            node_ = &search_node;
        }
        traverse(search_node);
    }

    void apply(osg::PositionAttitudeTransform &search_node) override {
        if (search_node.getName() == name_) {
            node_ = &search_node;
        }
        traverse(search_node);
    }

    void apply(osg::Geode &search_node) override {
        if (search_node.getName() == name_) {
            node_ = &search_node;
        }
        traverse(search_node);
    }

    void apply(osg::Node &search_node) override {
        if (search_node.getName() == name_) {
            node_ = &search_node;
        }
        traverse(search_node);
    }

    osg::Node *getNode() {
        return node_;
    }

    /**
     * @brief helper function
     * @warning the nodemask of the given node must be 1, or else nullptr will be returned
     * */
    static osg::Node *findNodeWithName(osg::Switch *root_node, const char name[]) {
        auto visitor = new NodeTreeSearch(name);
        root_node->accept(*visitor);
        auto node = visitor->getNode();

        if (!node) std::cout << "aha?" << std::endl;

        return node;
    }

private:
    osg::Node *node_;
    std::string name_;
};
}
#endif //POINTCLOUDVIEWER_NODETREESEARCH_H
