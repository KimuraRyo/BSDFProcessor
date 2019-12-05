// =================================================================== //
// Copyright (C) 2014-2019 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "GraphWidget.h"

#include <QtWidgets>
#include <QtCore/QMimeData>
#include <QtCore/QUrl>

#include <osgGA/TrackballManipulator>

#include "SceneUtil.h"

GraphWidget::GraphWidget(QWidget*           parent,
                         Qt::WindowFlags    f)
                         : OsgQWidget(parent, f),
                           graphScene_(nullptr)
{
    setAcceptDrops(true);

    osg::Camera* camera = new osg::Camera;
    camera->setViewport(0, 0, width(), height());
    camera->setClearColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));
    double aspectRatio = static_cast<double>(width()) / height();
    camera->setProjectionMatrixAsPerspective(30.0, aspectRatio, 0.00001, 100000.0);
    camera->setComputeNearFarMode(osg::Camera::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
    camera->setNearFarRatio(0.00005);
    viewer_->setCamera(camera);

    osgGA::TrackballManipulator* trackball = new osgGA::TrackballManipulator;
    trackball->setHomePosition(osg::Vec3d(0.0, 10.0, 0.0), osg::Vec3d(0.0, 0.0, 0.0), osg::Vec3d(0.0, 0.0, 1.0));
    trackball->home(0.0);
    trackball->setMinimumDistance(0.0001);
    trackball->setAllowThrow(false);
    viewer_->setCameraManipulator(trackball);

    actionResetCamera_ = new QAction(this);
    actionResetCamera_->setText(QApplication::translate("GraphWidget", "Reset camera position", 0));
    connect(actionResetCamera_, SIGNAL(triggered()), this, SLOT(resetCameraPosition()));

    actionCopyCameraSettings_ = new QAction(this);
    actionCopyCameraSettings_->setText(QApplication::translate("GraphWidget", "Copy camera settings", 0));
    connect(actionCopyCameraSettings_, SIGNAL(triggered()), this, SLOT(copyCameraSettings()));

    actionPasteCameraSettings_ = new QAction(this);
    actionPasteCameraSettings_->setText(QApplication::translate("GraphWidget", "Paste camera settings", 0));
    connect(actionPasteCameraSettings_, SIGNAL(triggered()), this, SLOT(pasteCameraSettings()));

    actionLogPlot_ = new QAction(this);
    actionLogPlot_->setText(QApplication::translate("GraphWidget", "Log plot", 0));
    actionLogPlot_->setCheckable(true);
    connect(actionLogPlot_, SIGNAL(toggled(bool)), this, SLOT(toggleLogPlot(bool)));
}

void GraphWidget::setGraphScene(GraphScene* scene)
{
    graphScene_ = scene;
    viewer_->setSceneData(graphScene_->getRoot());
}

void GraphWidget::copyCameraSettings()
{
    auto tm = dynamic_cast<osgGA::TrackballManipulator*>(viewer_->getCameraManipulator());
    if (!tm) return;

    osg::Vec3d pos, center, up;
    tm->getTransformation(pos, center, up);

    QString posStr = "CameraPosition "
                   + QString::number(pos.x()) + " "
                   + QString::number(pos.y()) + " "
                   + QString::number(pos.z());
    QString centerStr = "CameraCenter "
                      + QString::number(center.x()) + " "
                      + QString::number(center.y()) + " "
                      + QString::number(center.z());
    QString upStr = "CameraUp "
                  + QString::number(up.x()) + " "
                  + QString::number(up.y()) + " "
                  + QString::number(up.z());

    QClipboard* clipboard = QGuiApplication::clipboard();
    clipboard->setText(posStr + " " + centerStr + " " + upStr);
}

void GraphWidget::pasteCameraSettings()
{
    const QClipboard* clipboard = QApplication::clipboard();
    QStringList paramList = clipboard->text().split(' ', QString::SkipEmptyParts);

    osg::Vec3d pos, center, up;
    if (getParameters(paramList, "CameraPosition",  &pos) &&
        getParameters(paramList, "CameraCenter",    &center) &&
        getParameters(paramList, "CameraUp",        &up)) {
        auto tm = dynamic_cast<osgGA::TrackballManipulator*>(viewer_->getCameraManipulator());
        if (!tm) return;

        tm->setTransformation(pos, center, up);
    }
}

void GraphWidget::initializeGL()
{
    OsgQWidget::initializeGL();

    scene_util::displayGlInformation(graphicsWindow_);
}

void GraphWidget::resizeGL(int w, int h)
{
    OsgQWidget::resizeGL(w, h);

    if (graphScene_) {
        graphScene_->updateView(w, h);
    }
}

void GraphWidget::keyPressEvent(QKeyEvent* event)
{
    OsgQWidget::keyPressEvent(event);

    if (!graphScene_) return;

    auto tm = dynamic_cast<osgGA::TrackballManipulator*>(viewer_->getCameraManipulator());
    if (!tm) return;

    osg::Vec3 camPos, camCenter, camUp;
    viewer_->getCamera()->getViewMatrixAsLookAt(camPos, camCenter, camUp);
    osg::Vec3 camDir = camPos - camCenter;
    camDir.normalize();

    switch (event->key()) {
        case Qt::Key_Up: {
            osg::Vec3 rotAxis = camDir ^ camUp;
            osg::Quat rotQuat(lb::PI_D / 180.0, rotAxis);
            tm->setRotation(tm->getRotation() * rotQuat);

            break;
        }
        case Qt::Key_Down: {
            osg::Vec3 rotAxis = camDir ^ camUp;
            osg::Quat rotQuat(-lb::PI_D / 180.0, rotAxis);
            tm->setRotation(tm->getRotation() * rotQuat);

            break;
        }
        case Qt::Key_Left: {
            osg::Vec3 rotAxis(0.0, 0.0, 1.0);
            osg::Quat rotQuat(-lb::PI_D / 180.0, rotAxis);
            tm->setRotation(tm->getRotation() * rotQuat);

            break;
        }
        case Qt::Key_Right: {
            osg::Vec3 rotAxis(0.0, 0.0, 1.0);
            osg::Quat rotQuat(lb::PI_D / 180.0, rotAxis);
            tm->setRotation(tm->getRotation() * rotQuat);

            break;
        }
        case Qt::Key_Plus: {
            tm->setDistance(tm->getDistance() * 0.9);

            break;
        }
        case Qt::Key_Minus: {
            tm->setDistance(tm->getDistance() * 1.0 / 0.9);

            break;
        }
        default: {
            break;
        }
    }
}

void GraphWidget::mouseReleaseEvent(QMouseEvent* event)
{
    OsgQWidget::mouseReleaseEvent(event);

    if (event->button() == Qt::LeftButton && !mouseMoved_) {
        osg::Vec3 intersectPosition;
        osg::Node* pickedNode = scene_util::pickNode(viewer_, osg::Vec2(event->x(), height() - event->y()),
                                                     intersectPosition,
                                                     BRDF_MASK | SPECULAR_REFLECTANCE_MASK,
                                                     false);

        if (pickedNode) {
            lbInfo << "[GraphWidget::mouseReleaseEvent] " << pickedNode->getName();
            emit picked(intersectPosition);
        }
        else {
            emit clearPickedValue();
        }
    }
#ifdef __APPLE__
    else if (event->button() == Qt::RightButton && !mouseMoved_) {
        showContextMenu(event->globalPos());
    }
#endif
}

void GraphWidget::dragEnterEvent(QDragEnterEvent* event)
{
    if (event->mimeData()->hasUrls()) {
        event->acceptProposedAction();
    }
}

void GraphWidget::dropEvent(QDropEvent* event)
{
    QList<QUrl> urls = event->mimeData()->urls();
    QString fileName = urls.front().toLocalFile();

    emit fileDropped(fileName);
}

void GraphWidget::contextMenuEvent(QContextMenuEvent* event)
{
#ifndef __APPLE__
    if (mouseMoved_) return;

    showContextMenu(event->globalPos());
#endif

    update();
}

void GraphWidget::showContextMenu(const QPoint& pos)
{
    actionLogPlot_->setChecked(graphScene_->isLogPlot());

    if (graphScene_->isLogPlotAcceptable()) {
        actionLogPlot_->setEnabled(true);
    }
    else {
        actionLogPlot_->setDisabled(true);
    }

    QMenu menu(this);
    menu.addAction(actionResetCamera_);
    menu.addAction(actionCopyCameraSettings_);
    menu.addAction(actionPasteCameraSettings_);
    menu.addSeparator();
    menu.addAction(actionLogPlot_);
    menu.exec(pos);
}

bool GraphWidget::getParameters(const QStringList& paramList, const QString& name, osg::Vec3d* params)
{
    int index = paramList.indexOf(name);
    if (index == -1) {
        lbError << "[GraphWidget::getParameters] Parameter not found: " << name.toStdString();
        return false;
    }

    if (index + 3 >= paramList.size()) {
        lbError << "[GraphWidget::getParameters] Invalid format.";
        return false;
    }

    bool okX, okY, okZ;
    double x = paramList.at(index + 1).toDouble(&okX);
    double y = paramList.at(index + 2).toDouble(&okY);
    double z = paramList.at(index + 3).toDouble(&okZ);
    if (okX && okY && okZ) {
        params->set(x, y, z);
    }
    else {
        lbError << "[GraphWidget::getParameters] Invalid format.";
        return false;
    }

    return true;
}
