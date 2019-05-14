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

#include <stdio.h>

#include <QtGui/QKeyEvent>
#include <QtGui/QPainter>
#include <QtCore/QFile>
#include <QtCore/QTextStream>

#include <osg/Timer>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/LineWidth>
#include <osg/PolygonMode>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osg/ComputeBoundsVisitor>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TerrainManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgDB/ReadFile>

#include "OSGWidget.h"
#include "Singleton.h"
#include "NodeCallback.h"
#include "NodeTreeInfo.h"
#include "NodeTreeSearch.h"
#include "CoorConv.hpp"

using namespace osgHelper;

osg::Vec3d utm2llh(const osg::Vec3d &enu) {
    WGS84Corr latlon;
    UTMXYToLatLon(enu.x(), enu.y(), 50, false, latlon);

    return {RadToDeg(latlon.lat), RadToDeg(latlon.log), enu.z()};
}

OSGWidget::OSGWidget(QWidget *parent, Qt::WindowFlags f)
        : QOpenGLWidget(parent, f),
          _graphicsWindow(new osgViewer::GraphicsWindowEmbedded(this->x(), this->y(), this->width(), this->height())),
          _viewer(nullptr),
          root_node_(nullptr),
          text_node_(nullptr),
          is_testing_(true),
          update_timer_(new QTimer(this)) {
    // enable keypress event
    this->setFocusPolicy(Qt::StrongFocus);

    connect(update_timer_.data(), &QTimer::timeout, this, &OSGWidget::updateScene);
}

void OSGWidget::init() {
    initSceneGraph();
    initHelperNode();
    initCamera();

    initTrace();
    cur_position = trace_vec_[0];

    startTimer(1000 / 60.f);  // 60hz
}

void OSGWidget::initSceneGraph() {
    root_node_ = new osg::Switch;
    root_node_->setName(root_node_name);

    osg::ref_ptr<osg::Switch> point_cloud_node = new osg::Switch;
    point_cloud_node->setName(point_cloud_node_name);
    root_node_->addChild(point_cloud_node);

    osg::ref_ptr<osg::Switch> dsm_node = new osg::Switch;
    dsm_node->setName(dsm_node_name);
    root_node_->addChild(dsm_node);

    osg::ref_ptr<osg::Switch> building_node = new osg::Switch;
    building_node->setName(building_node_name);
    root_node_->addChild(building_node);

    osg::ref_ptr<osg::PositionAttitudeTransform> uav_node = new osg::PositionAttitudeTransform;
    uav_node->setName(uav_node_name);
    {
        osg::ref_ptr<osg::Node> plane = osgDB::readNodeFile("./model/uav.osgb");
        if (plane.valid()) {
            osg::ref_ptr<osg::MatrixTransform> mt_node = new osg::MatrixTransform;
            mt_node->setMatrix(osg::Matrixd::scale(0.1, 0.1, 0.1));

            mt_node->addChild(plane);
            uav_node->addChild(mt_node);
        } else {
            osg::ref_ptr<osg::Geode> geode = new osg::Geode;
            geode->setName("UAV");

            osg::ref_ptr<osg::ShapeDrawable> point_sphere = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3d(), 0.5f));
            point_sphere->setColor(osg::Vec4(1.0, 1.0, 0.0, 1.0));
            geode->addDrawable(point_sphere);

            uav_node->addChild(geode);
        }
    }
    root_node_->addChild(uav_node);

    osg::ref_ptr<osg::Camera> hud_node = createHUD();
    hud_node->setName(hud_node_name);
    {
        text_node_ = new osg::Switch;
        text_node_->setName(text_node_name);
        hud_node->addChild(text_node_);
    }
    root_node_->addChild(hud_node);

    osg::ref_ptr<osg::Switch> helper_node = new osg::Switch;
    helper_node->setName(helper_node_name);
    root_node_->addChild(helper_node);

    {
        osg::ref_ptr<osg::StateSet> ss = new osg::StateSet;
        ss->setMode(GL_LIGHTING, osg::StateAttribute::ON);
        osg::ref_ptr<osg::Light> light = new osg::Light;
        light->setAmbient(osg::Vec4(.8f, .5f, .3f, .8f));
        ss->setAttribute(light, osg::StateAttribute::ON);
        ss->setMode(GL_BLEND, osg::StateAttribute::ON);
        ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        ss->setMode(GL_MULTISAMPLE_ARB, osg::StateAttribute::ON);

        dsm_node->setStateSet(ss.get());
    }
}

void OSGWidget::initCamera() {
    _viewer = new osgViewer::Viewer;

    float aspectRatio = static_cast<float>(this->width()) / static_cast<float>(this->height());
    osg::ref_ptr<osg::Camera> camera = _viewer->getCamera();
    camera->setViewport(0, 0, this->width(), this->height());
    //    camera->setClearColor(osg::Vec4(0.2, 0.2, 0.6, 1.0));
    camera->setProjectionMatrixAsPerspective(30.f, aspectRatio, 1.f, 10000.f);
    camera->setGraphicsContext(_graphicsWindow);
    camera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
    camera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    camera->setProjectionMatrixAsPerspective(30.f,
                                             static_cast<double>(this->width()) / static_cast<double>(this->height()),
                                             1.0, 1000.0);
    camera->setNearFarRatio(0.0000002);
    camera->setComputeNearFarMode(osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
//    camera->setClearColor(osg::Vec4(0.84313, 0.84313, 0.89804, 1.0));

    _viewer->addEventHandler(new osgViewer::StatsHandler);
    _viewer->addEventHandler(new osgGA::StateSetManipulator(camera->getStateSet()));
    _viewer->addEventHandler(new NodeTreeHandler(root_node_));

    //for outline effects
    {
        osg::DisplaySettings::instance()->setMinimumNumStencilBits(1);
        unsigned int clearMask = camera->getClearMask();
        camera->setClearMask(clearMask | GL_STENCIL_BUFFER_BIT);
        camera->setClearStencil(0);
    }

    _terrainMani = new osgGA::TerrainManipulator;
    _terrainMani->setAllowThrow(false);
    _terrainMani->setAutoComputeHomePosition(true);

    _trackballMani = new osgGA::TrackballManipulator;
    _trackballMani->setAllowThrow(false);
    _trackballMani->setVerticalAxisFixed(true);
    _viewer->setCameraManipulator(_trackballMani.get());

    _viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
    _viewer->setSceneData(root_node_);
}

void OSGWidget::initHelperNode() {
    static osg::ref_ptr<osg::Switch> helper_node = dynamic_cast<osg::Switch *>(
            NodeTreeSearch::findNodeWithName(root_node_, helper_node_name));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;

    float radius = 10.0;
    osg::Vec3d axis_center;
    vertices->push_back(axis_center);
    vertices->push_back(axis_center + osg::Vec3(radius, 0, 0));
    vertices->push_back(axis_center + osg::Vec3(0, radius, 0));
    vertices->push_back(axis_center + osg::Vec3(0, 0, radius));

    colors->push_back(osg::Vec3(1, 0, 0));
    colors->push_back(osg::Vec3(0, 1, 0));
    colors->push_back(osg::Vec3(0, 0, 1));

    geom->setVertexArray(vertices);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

    osg::ref_ptr<osg::DrawElementsUInt> line1 = new osg::DrawElementsUInt(osg::DrawElementsUInt::LINES, 2);
    (*line1)[0] = 0;
    (*line1)[1] = 1;
    osg::ref_ptr<osg::DrawElementsUInt> line2 = new osg::DrawElementsUInt(osg::DrawElementsUInt::LINES, 2);
    (*line2)[0] = 0;
    (*line2)[1] = 2;
    osg::ref_ptr<osg::DrawElementsUInt> line3 = new osg::DrawElementsUInt(osg::DrawElementsUInt::LINES, 2);
    (*line3)[0] = 0;
    (*line3)[1] = 3;

    geom->addPrimitiveSet(line1);
    geom->addPrimitiveSet(line2);
    geom->addPrimitiveSet(line3);

    osg::ref_ptr<osg::StateSet> state_set = geode->getOrCreateStateSet();
    osg::ref_ptr<osg::LineWidth> line_width = new osg::LineWidth(3.0);
    state_set->setAttributeAndModes(line_width);
    geode->addDrawable(geom);

    helper_node->addChild(geode);
}

osg::Camera *OSGWidget::createHUD() {
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;

    camera->setProjectionMatrix(osg::Matrix::ortho2D(0, 1280, 0, 1024));
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    camera->setRenderOrder(osg::Camera::POST_RENDER);
    camera->setAllowEventFocus(false);

    return camera.release();
}

void OSGWidget::readDataFromFile(const QFileInfo &file_info) {
    static osg::ref_ptr<osg::Switch> point_cloud_node = dynamic_cast<osg::Switch *>(
            NodeTreeSearch::findNodeWithName(root_node_, point_cloud_node_name));

    osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(file_info.filePath().toStdString());
    osg::ref_ptr<osg::Geode> bbox = calculateBBoxForModel(node);

    osg::ref_ptr<osg::MatrixTransform> matrix = new osg::MatrixTransform;
    matrix->addChild(node);
    matrix->addChild(bbox);
    matrix->setUpdateCallback(new NodeCallback());

    point_cloud_node->removeChildren(0, point_cloud_node->getNumChildren());
    point_cloud_node->addChild(matrix);
}

osg::Geode *OSGWidget::calculateBBoxForModel(osg::Node *node) const {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    osg::ComputeBoundsVisitor bounds_visitor;
    node->accept(bounds_visitor);
    osg::BoundingBox bb = bounds_visitor.getBoundingBox();
    float lengthX = bb.xMax() - bb.xMin();
    float lengthY = bb.yMax() - bb.yMin();
    float lengthZ = bb.zMax() - bb.zMin();
    osg::Vec3 center = bb.center();
    std::cout << "model center: " << center << std::endl;

    osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(new osg::Box(center, lengthX, lengthY, lengthZ));
    drawable->setColor(osg::Vec4(1.0, 1.0, 0.0, 1.0));

    osg::ref_ptr<osg::StateSet> state_set = drawable->getOrCreateStateSet();
    osg::ref_ptr<osg::PolygonMode> polygon = new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK,
                                                                  osg::PolygonMode::LINE);
    state_set->setAttributeAndModes(polygon);
    osg::ref_ptr<osg::LineWidth> line_width = new osg::LineWidth(3.0);
    state_set->setAttribute(line_width);
    geode->addDrawable(drawable);

    return geode.release();
}

osgGA::EventQueue *OSGWidget::getEventQueue() const {
    osgGA::EventQueue *eventQueue = _graphicsWindow->getEventQueue();

    if (eventQueue) {
        return eventQueue;
    } else {
        throw std::runtime_error("Unable to obtain valid event queue");
    }
}

void OSGWidget::paintEvent(QPaintEvent * /* paintEvent */) {
    this->makeCurrent();

//    QPainter painter(this);
//    painter.setRenderHint(QPainter::Antialiasing);

    this->paintGL();

//    painter.end();

    this->doneCurrent();
}

void OSGWidget::paintGL() {
    _viewer->frame();
}

void OSGWidget::onResize(int width, int height) {
    _viewer->getCamera()->setViewport(0, 0, this->width(), this->height());
}

void OSGWidget::resizeGL(int width, int height) {
    this->getEventQueue()->windowResize(this->x(), this->y(), width, height);
    _graphicsWindow->resized(this->x(), this->y(), width, height);

    this->onResize(width, height);
}

void OSGWidget::keyPressEvent(QKeyEvent *event) {
    char key_c = event->text().toStdString()[0];
//    LOG_INFO << "OSGWidget get key: " << key_c;

    this->getEventQueue()->keyPress(osgGA::GUIEventAdapter::KeySymbol(key_c));
}

void OSGWidget::mouseMoveEvent(QMouseEvent *event) {
    this->getEventQueue()->mouseMotion(static_cast<float>(event->x()), static_cast<float>(event->y()));
}

void OSGWidget::mousePressEvent(QMouseEvent *event) {
    // 1 = left mouse button
    // 2 = middle mouse button
    // 3 = right mouse button

    unsigned int button = 0;

    switch (event->button()) {
        case Qt::LeftButton:
            button = 1;
            break;

        case Qt::MiddleButton:
            button = 2;
            break;

        case Qt::RightButton:
            button = 3;
            break;

        default:
            break;
    }

    Qt::KeyboardModifiers mod = event->modifiers();
    quint16 modkeyosg = 0;
    if (mod & Qt::ShiftModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_SHIFT;
    if (mod & Qt::ControlModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_CTRL;
    if (mod & Qt::AltModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_ALT;
    if (mod & Qt::MetaModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_META;

    osgGA::GUIEventAdapter *adapter = this->getEventQueue()->mouseButtonPress(static_cast<float>(event->x()),
                                                                              static_cast<float>(event->y()),
                                                                              button);
    adapter->setModKeyMask(modkeyosg);
}

void OSGWidget::mouseReleaseEvent(QMouseEvent *event) {
    // 1 = left mouse button
    // 2 = middle mouse button
    // 3 = right mouse button

    unsigned int button = 0;

    switch (event->button()) {
        case Qt::LeftButton:
            button = 1;
            break;

        case Qt::MiddleButton:
            button = 2;
            break;

        case Qt::RightButton:
            button = 3;
            break;

        default:
            break;
    }
    Qt::KeyboardModifiers mod = event->modifiers();
    quint16 modkeyosg = 0;
    if (mod & Qt::ShiftModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_SHIFT;
    if (mod & Qt::ControlModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_CTRL;
    if (mod & Qt::AltModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_ALT;
    if (mod & Qt::MetaModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_META;

    osgGA::GUIEventAdapter *adapter = this->getEventQueue()->mouseButtonRelease(static_cast<float>(event->x()),
                                                                                static_cast<float>(event->y()),
                                                                                button);
    adapter->setModKeyMask(modkeyosg);
}

void OSGWidget::wheelEvent(QWheelEvent *event) {
    event->accept();
    int delta = event->delta();

    osgGA::GUIEventAdapter::ScrollingMotion motion = delta > 0 ? osgGA::GUIEventAdapter::SCROLL_UP
                                                               : osgGA::GUIEventAdapter::SCROLL_DOWN;
    Qt::KeyboardModifiers mod = event->modifiers();
    quint16 modkeyosg = 0;
    if (mod & Qt::ShiftModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_SHIFT;
    if (mod & Qt::ControlModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_CTRL;
    if (mod & Qt::AltModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_ALT;
    if (mod & Qt::MetaModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_META;

    osgGA::GUIEventAdapter *adapter = this->getEventQueue()->mouseScroll(motion);
    adapter->setModKeyMask(modkeyosg);
}

void OSGWidget::mouseDoubleClickEvent(QMouseEvent *event) {
    // 1 = left mouse button
    // 2 = middle mouse button
    // 3 = right mouse button

    unsigned int button = 0;

    switch (event->button()) {
        case Qt::LeftButton:
            button = 1;
            break;

        case Qt::MiddleButton:
            button = 2;
            break;

        case Qt::RightButton:
            button = 3;
            break;

        default:
            break;
    }
    Qt::KeyboardModifiers mod = event->modifiers();
    quint16 modkeyosg = 0;
    if (mod & Qt::ShiftModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_SHIFT;
    if (mod & Qt::ControlModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_CTRL;
    if (mod & Qt::AltModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_ALT;
    if (mod & Qt::MetaModifier)
        modkeyosg |= osgGA::GUIEventAdapter::MODKEY_META;

    osgGA::GUIEventAdapter *adapter = this->getEventQueue()->mouseDoubleButtonPress(static_cast<float>(event->x()),
                                                                                    static_cast<float>(event->y()),
                                                                                    button);
    adapter->setModKeyMask(modkeyosg);
}

bool OSGWidget::event(QEvent *event) {
    bool handled = QOpenGLWidget::event(event);
    // This ensures that the OSG widget is always going to be repainted after the
    // user performed some interaction. Doing this in the event handler ensures
    // that we don't forget about some event and prevents duplicate code.
    switch (event->type()) {
        case QEvent::KeyPress:
        case QEvent::KeyRelease:
        case QEvent::MouseButtonDblClick:
        case QEvent::MouseButtonPress:
        case QEvent::MouseButtonRelease:
        case QEvent::MouseMove:
        case QEvent::Wheel:
            //        this->update();
            break;
        default:
            break;
    }

    return handled;
}

void OSGWidget::timerEvent(QTimerEvent *) {
    update();
}

void OSGWidget::home() {
    _viewer->home();
}

void OSGWidget::trackballCenterOn(double x, double y, double z) {
    _trackballMani->setCenter(osg::Vec3d(x, y, z));
    _trackballMani->setDistance(700);
}

void OSGWidget::updateUAVPose(Point position) {
    static osg::ref_ptr<osg::PositionAttitudeTransform> uav_node = dynamic_cast<osg::PositionAttitudeTransform *>(
            NodeTreeSearch::findNodeWithName(root_node_, uav_node_name));

    cur_position = position;
    uav_node->setPosition(position);

    std::stringstream ss;
    ss << position;
    std::string position_str = ss.str();

    //update text
    {
        text_node_->removeChildren(0, text_node_->getNumChildren());

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->setName(positon_geode_name);
        // turn lighting off for the text and disable depth test to ensure it's always on top.
        osg::ref_ptr<osg::StateSet> state_set = geode->getOrCreateStateSet();
        state_set->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

        //position
        {
            osg::Vec3 loc(20.0f, 20.0f, 0.0f);

            osg::ref_ptr<osgText::Text> text = new osgText::Text;
            geode->addDrawable(text);

            text->setPosition(loc);
            text->setCharacterSize(20);
            text->setText("UAV Pos: " + position_str);
        }
        text_node_->addChild(geode);
    }
}

void OSGWidget::updatePointCloud() {
    static osg::ref_ptr<osg::Switch> point_cloud_node = dynamic_cast<osg::Switch *>(
            NodeTreeSearch::findNodeWithName(root_node_, point_cloud_node_name));

    cur_points = core::Singleton<Array>::getInstance().getValue();

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;

    for (const auto &point : cur_points) {
        vertices->push_back(point);
        colors->push_back(calculateColorForPoint(point));
    }

    geom->setVertexArray(vertices);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));

    geode->addDrawable(geom);
    point_cloud_node->addChild(geode);

    if (point_cloud_node->getNumChildren() >= 10) {
        point_cloud_node->removeChildren(0, 3);
    }
}

osg::Vec3d OSGWidget::calculateColorForPoint(const osg::Vec3d &point) const {
    static std::vector<osg::Vec3> Colors =
            {osg::Vec3(0.0, 0.0, 1.0), osg::Vec3(0.0, 0.2, 1.0), osg::Vec3(0.0, 0.4, 1.0), osg::Vec3(0.0, 0.6, 1.0),
             osg::Vec3(0.0, 0.8, 1.0), osg::Vec3(0.0, 1.0, 1.0),
             osg::Vec3(0.0, 1.0, 0.8), osg::Vec3(0.0, 1.0, 0.6), osg::Vec3(0.0, 1.0, 0.4), osg::Vec3(0.0, 1.0, 0.2),
             osg::Vec3(0.0, 1.0, 0.0), osg::Vec3(0.2, 1.0, 0.0),
             osg::Vec3(0.4, 1.0, 0.0), osg::Vec3(0.6, 1.0, 0.0), osg::Vec3(0.8, 1.0, 0.0), osg::Vec3(1.0, 1.0, 0.0),
             osg::Vec3(1.0, 0.8, 0.0), osg::Vec3(1.0, 0.6, 0.0),
             osg::Vec3(1.0, 0.4, 0.0), osg::Vec3(1.0, 0.2, 0.0), osg::Vec3(1.0, 0.0, 0.0)
            };
    const int height_range = 5;

    double height = std::abs(point.z() - cur_position.z());
    int range = static_cast<int>(height) / height_range;
    if (range >= Colors.size()) range = Colors.size() - 1;
    if (range == 0) range = 1;

    return Colors[Colors.size() - range];
}

void OSGWidget::initTrace() {
    //test trace
    trace_vec_.clear();
    trace_vec_.shrink_to_fit();

    int height = 350;
    trace_vec_.emplace_back(201952, 2502601, height);
    trace_vec_.emplace_back(201889, 2502756, height);
    trace_vec_.emplace_back(202948, 2502650, height);
    trace_vec_.emplace_back(202804, 2501826, height);
    trace_vec_.emplace_back(202062, 2501918, height);
    trace_vec_.emplace_back(202085, 2502578, height);
    trace_vec_.emplace_back(202804, 2502493, height);
    trace_vec_.emplace_back(202921, 2502116, height);
    trace_vec_.emplace_back(202128, 2502150, height);

    std::vector<osg::Vec3d> intense_trace;
    int inter = 50;

    for (int i = 0; i < trace_vec_.size() - 1; ++i) {
        osg::Vec3d p1 = trace_vec_[i];
        osg::Vec3d p2 = trace_vec_[i + 1];

        intense_trace.push_back(p1);

        osg::Vec3d p12 = p2 - p1;
        double distance = p12.length();
        p12.normalize();
        int steps = static_cast<int>(distance / inter);
        int cnt = 0;
        while (cnt < steps) {
            cnt++;

            osg::Vec3d temp = p1 + p12 * cnt * inter;
            intense_trace.push_back(temp);
        }
    }

    std::swap(trace_vec_, intense_trace);
}

void OSGWidget::loadDSM(const std::string &file_path) {
    static osg::ref_ptr<osg::Switch> dsm_node =
            dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(root_node_, dsm_node_name));

    osg::ref_ptr<osg::Node> dsm = osgDB::readNodeFile(file_path);

    if (!dsm.valid()) {
        std::cout << "failed to load dsm file: " << file_path << std::endl;
        return;
    }

    dsm_node->removeChildren(0, dsm_node->getNumChildren());
    dsm_node->addChild(dsm);

    osg::BoundingSphere bs = dsm->getBound();
    std::cout << "dsm center " << bs.center().x() << " "
              << bs.center().y() << " " << bs.center().z() << std::endl;

    _terrainMani->setHomePosition(cur_position + osg::Vec3(0.0, 0.0, 1000.0), cur_position, osg::Vec3(0, 1, 0));
    _viewer->setCameraManipulator(_terrainMani.get());

    updateUAVPose(cur_position);
}

void OSGWidget::loadBuildings(const std::string &file_path) {
    static osg::ref_ptr<osg::Switch> building_node =
            dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(root_node_, building_node_name));

    QString file_path_str = QString::fromStdString(file_path);
    QFile file(file_path_str);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;

    QFileInfo fileInfo(file_path_str);
    if (!fileInfo.exists()) return;
    QString file_dir = fileInfo.path();

    osg::ref_ptr<osg::PositionAttitudeTransform> building_trans_node = new osg::PositionAttitudeTransform;
    building_trans_node->setName(building_trans_node_name);
    building_trans_node->setPosition(osg::Vec3d(0, 0, 300));
    building_node->removeChildren(0, building_node->getNumChildren());
    building_node->addChild(building_trans_node);

    QTextStream in(&file);
    while (!in.atEnd()) {
        QString lineStr = in.readLine();
        QStringList sections = lineStr.split(QRegExp("[, \t]"));

        if (sections.size() != 5) continue;
        QString nameStr = sections[0];
        double x = sections[1].toDouble();
        double y = sections[2].toDouble();
        double z = sections[3].toDouble();
        double r = sections[4].toDouble();
        QString node_file_path = file_dir + "/" + nameStr;

        osg::ref_ptr<osg::PagedLOD> page_lod = new osg::PagedLOD;
        page_lod->setName(nameStr.toStdString());
        page_lod->setCenter(osg::Vec3d(x, y, z));
        page_lod->setCenterMode(osg::LOD::USER_DEFINED_CENTER);
        page_lod->setRadius(r);
        page_lod->setFileName(0, node_file_path.toStdString());
        page_lod->setRange(0, 0, 8000.0f);
        page_lod->setRangeMode(osg::LOD::DISTANCE_FROM_EYE_POINT);

        building_trans_node->addChild(page_lod.release());
    }
}

void OSGWidget::activeTraceRefresh(bool is_active) {
    is_testing_ = false;

    if (is_active) {
        beginTraceRefresh();
    } else {
        finishTraceRefresh();
    };
}

void OSGWidget::activeTestRefresh(bool is_active) {
    is_testing_ = true;

    if (is_active) {
        beginTraceRefresh();
    } else {
        finishTraceRefresh();
    }
}

void OSGWidget::beginTraceRefresh() {
    //for setViewMatrixAsLookAt() to work
    _viewer->setCameraManipulator(nullptr);

    update_timer_->start(30);
}

void OSGWidget::finishTraceRefresh() {
    _terrainMani->setHomePosition(cur_position + osg::Vec3(0.0, 0.0, 1000.0), cur_position, osg::Vec3(0, 1, 0));
    _viewer->setCameraManipulator(_terrainMani);

    update_timer_->stop();
}

void OSGWidget::updateScene() {

    // tips
    {
        std::cout << "cur_position: " << cur_position << " point num: " << cur_points.size() << std::endl;
    }

    //calculate intersection for current dsm height and building id
    Result result = intersectionCheck();

    //match the point cloud data
//    pointCloudMatch(result);

//    postResult(result);
}

Result OSGWidget::intersectionCheck() {
    static osg::ref_ptr<osg::Switch> model_node =
            dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(root_node_, model_node_name));

    osg::Vec3d start = cur_position;
    start.z() += 1000;
    osg::Vec3d end = cur_position;
    end.z() -= 1000;

    int id = -1;
    double build_area = 0;
    osg::Vec3d ground_point = cur_position;
    ground_point.z() = 0;

    osg::Vec3d llh = utm2llh(cur_position);

    osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(start, end);
    osgUtil::IntersectionVisitor iv(intersector.get());

    model_node->setNodeMask(0);
    root_node_->accept(iv);
    model_node->setNodeMask(1);

    if (intersector->containsIntersections()) {
        auto intersections = intersector->getIntersections();

        for (const auto &intersection : intersections) {
            osg::NodePath node_path = intersection.nodePath;

            bool is_building = false;
            bool is_dsm = false;
            for (const auto &node : node_path) {
                if (node->getName() == building_node_name) is_building = true;
                if (node->getName() == dsm_node_name) is_dsm = true;
            }

            if (is_building) //building node
            {
                osg::ref_ptr<osg::Node> node = node_path.back();

                osg::ref_ptr<osg::Geode> geode = dynamic_cast<osg::Geode *>(node.get());
                if (geode.valid()) {
                    osg::ref_ptr<osg::Drawable> drawable = geode->getDrawable(0);
                    if (drawable.valid()) {
                        osg::ref_ptr<osg::Geometry> geom = dynamic_cast<osg::Geometry *>(drawable.get());
                        geom->setUseDisplayList(false);
                        geom->setUseVertexBufferObjects(true);
                        osg::ref_ptr<osg::Vec3Array> colors = dynamic_cast<osg::Vec3Array *>(geom->getColorArray());
                        auto &color = colors->back();
                        color.set(0.0, 1.0, 0.0);
                        colors->dirty();

                        osg::ref_ptr<osg::Vec3Array> points = dynamic_cast<osg::Vec3Array *>(geom->getVertexArray());


                    }
                }

                std::string node_name = node->getName();
                id = std::stoi(node_name);
            }

            if (is_dsm) //dsm node
            {
                ground_point = intersection.getWorldIntersectPoint();
            }
        }

        std::cout << "id: " << id << "   ground_point: " << ground_point << std::endl;
    }

    Result result(id, ground_point, llh, build_area, 0, false);

    return result;
}
