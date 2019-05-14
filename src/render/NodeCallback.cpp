#include <iostream>

#include <osg/MatrixTransform>

#include "Common.h"
#include "NodeCallback.h"

using namespace osgHelper;

NodeCallback::NodeCallback() :
    angle_(0),
    radius_(10),
    center_(osg::Vec3d(10.0, 10.0, 0.0)) {

}

NodeCallback::~NodeCallback() = default;

void NodeCallback::operator()(osg::Node *node, osg::NodeVisitor *nv) {

    osg::ref_ptr<osg::MatrixTransform> mt = dynamic_cast<osg::MatrixTransform*>(node);
    if(!mt.valid()) return;

    double direction = angle_ - osg::PI_2;
    osg::Vec3d location = center_;
    location.x() += radius_ * cos(angle_);
    location.y() += radius_ * sin(angle_);

    //matrix rotate
    {

//        osg::Matrix mx = osg::Matrix(osg::Matrix::rotate(direction,0.0f,0.0f,1.0f)
//                * osg::Matrix::translate(location));

        osg::Matrix mx = osg::Matrix(cos(direction),  sin(direction), 0, 0,
                                     -sin(direction), cos(direction), 0, 0,
                                     0, 0, 1, 0,
                                     location.x(), location.y(), location.z(), 1);

        mt->setMatrix(mx);
    }

    //coordinate axis rotate(equal to the former)
    {
        osg::Vec3d axis_x = osg::Vec3d(cos(direction), sin(direction), 0.0);
        osg::Vec3d axis_y = osg::Vec3d(cos(angle_), sin(angle_), 0.0);
        osg::Vec3d axis_z = osg::Vec3d(0.0, 0.0, 1.0);

        osg::Matrix mx = osg::Matrix(axis_x.x(),  axis_x.y(), axis_x.z(), 0,
                                     axis_y.x(),  axis_y.y(), axis_y.z(), 0,
                                     axis_z.x(),  axis_z.y(), axis_z.z(), 0,
                                     location.x(), location.y(), location.z(), 1);

        mt->setMatrix(mx);
    }

    angle_ += 0.01;
    traverse(node,nv);
}
