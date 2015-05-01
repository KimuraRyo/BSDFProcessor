// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "GraphWidget.h"

#include <QtWidgets>
#include <QtCore/QMimeData>
#include <QtCore/QUrl>

#include <osgDB/WriteFile>
#include <osgGA/TrackballManipulator>

#include "GraphScene.h"
#include "SceneUtil.h"

GraphWidget::GraphWidget(const QGLFormat&   format,
                         QWidget*           parent,
                         const QGLWidget*   shareWidget,
                         Qt::WindowFlags    f,
                         bool               forwardKeyEvents)
                         : osgQt::GLWidget(format, parent, shareWidget, f, forwardKeyEvents),
                           graphScene_(0),
                           movedMouse_(false)
{
    setAcceptDrops(true);
    setMinimumSize(1, 1);

    actionResetCamera_ = new QAction(this);
    actionResetCamera_->setText(QApplication::translate("GraphWidget", "Reset camera position", 0));
    connect(actionResetCamera_, SIGNAL(triggered()), this, SLOT(resetCameraPosition()));
}

void GraphWidget::resizeEvent(QResizeEvent* event)
{
    osgQt::GLWidget::resizeEvent(event);
    
    if (graphScene_) {
        graphScene_->updateView(event->size().width(), event->size().height());
    }
}

void GraphWidget::keyPressEvent(QKeyEvent* event)
{
    osgQt::GLWidget::keyPressEvent(event);

    if (!graphScene_) return;

    osgGA::CameraManipulator* cm = graphScene_->getCameraManipulator();
    osgGA::TrackballManipulator* tm = dynamic_cast<osgGA::TrackballManipulator*>(cm);

    osg::Vec3 camPos, camCenter, camUp;
    graphScene_->getCamera()->getViewMatrixAsLookAt(camPos, camCenter, camUp);
    osg::Vec3 camDir = camPos - camCenter;
    camDir.normalize();

    switch (event->key()) {
        case Qt::Key_Up: {
            if (!tm) break;
            
            osg::Vec3 rotAxis = camDir ^ camUp;
            osg::Quat rotQuat(M_PI / 180.0, rotAxis);
            tm->setRotation(tm->getRotation() * rotQuat);

            break;
        }
        case Qt::Key_Down: {
            if (!tm) break;

            osg::Vec3 rotAxis = camDir ^ camUp;
            osg::Quat rotQuat(-M_PI / 180.0, rotAxis);
            tm->setRotation(tm->getRotation() * rotQuat);

            break;
        }
        case Qt::Key_Left: {
            if (!tm) break;

            osg::Vec3 rotAxis(0.0, 0.0, 1.0);
            osg::Quat rotQuat(-M_PI / 180.0, rotAxis);
            tm->setRotation(tm->getRotation() * rotQuat);

            break;
        }
        case Qt::Key_Right: {
            if (!tm) break;

            osg::Vec3 rotAxis(0.0, 0.0, 1.0);
            osg::Quat rotQuat(M_PI / 180.0, rotAxis);
            tm->setRotation(tm->getRotation() * rotQuat);

            break;
        }
        case Qt::Key_Plus: {
            if (!tm) break;

            tm->setDistance(tm->getDistance() * 0.9);

            break;
        }
        case Qt::Key_Minus: {
            if (!tm) break;

            tm->setDistance(tm->getDistance() * 1.0 / 0.9);

            break;
        }
        default: {
            break;
        }
    }
}

void GraphWidget::keyReleaseEvent(QKeyEvent* event)
{
    osgQt::GLWidget::keyReleaseEvent(event);
}

void GraphWidget::mouseMoveEvent(QMouseEvent* event)
{
    osgQt::GLWidget::mouseMoveEvent(event);
    movedMouse_ = true;
}

void GraphWidget::mousePressEvent(QMouseEvent* event)
{
    osgQt::GLWidget::mousePressEvent(event);
    movedMouse_ = false;
}

void GraphWidget::mouseReleaseEvent(QMouseEvent* event)
{
    osgQt::GLWidget::mouseReleaseEvent(event);

    if (event->button() == Qt::LeftButton && !movedMouse_) {
        osgViewer::GraphicsWindow::Views views;
        getGraphicsWindow()->getViews(views);

        osg::Vec3 intersectPosition;
        osg::Node* pickedNode = scene_util::pickNode(views.front(), osg::Vec2(event->x(), height() - event->y()),
                                                     intersectPosition,
                                                     BRDF_MASK | SPECULAR_REFLECTANCE_MASK,
                                                     false);

        if (pickedNode) {
            //std::cout << "[GraphWidget::mouseReleaseEvent] " << pickedNode->getName() << std::endl;
            emit picked(intersectPosition);
        }
        else {
            emit clearPickedValue();
        }
    }
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

void GraphWidget::wheelEvent(QWheelEvent* event)
{
    osgQt::GLWidget::wheelEvent(event);
    movedMouse_ = true;
}

void GraphWidget::contextMenuEvent(QContextMenuEvent* event)
{
    osgQt::GLWidget::contextMenuEvent(event);

    if (movedMouse_) return;

    QMenu menu(this);
    menu.addAction(actionResetCamera_);
    menu.exec(event->globalPos());
}
