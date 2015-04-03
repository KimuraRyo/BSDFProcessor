// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "RenderingWidget.h"

#include <QtWidgets>
#include <QtCore/QMimeData>
#include <QtCore/QUrl>

#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgUtil/Optimizer>

#include "RenderingScene.h"
#include "SceneUtil.h"

RenderingWidget::RenderingWidget(const QGLFormat&   format,
                                 QWidget*           parent,
                                 const QGLWidget*   shareWidget,
                                 Qt::WindowFlags    f,
                                 bool               forwardKeyEvents)
                                 : osgQt::GLWidget(format, parent, shareWidget, f, forwardKeyEvents),
                                   renderingScene_(0),
                                   movedMouse_(false)
{
    //setAcceptDrops(true);
    setMinimumSize(1, 1);

    actionResetCamera_ = new QAction(this);
    actionResetCamera_->setText(QApplication::translate("RenderingWidget", "Reset camera position", 0));
    connect(actionResetCamera_, SIGNAL(triggered()), this, SLOT(resetCameraPosition()));

    actionShapeSphere_ = new QAction(this);
    actionShapeSphere_->setText(QApplication::translate("RenderingWidget", "Sphere", 0));
    connect(actionShapeSphere_, SIGNAL(triggered()), this, SLOT(showSphere()));

    actionShapeCylinder_ = new QAction(this);
    actionShapeCylinder_->setText(QApplication::translate("RenderingWidget", "Cylinder", 0));
    connect(actionShapeCylinder_, SIGNAL(triggered()), this, SLOT(showCylinder()));

    actionShapeBox_ = new QAction(this);
    actionShapeBox_->setText(QApplication::translate("RenderingWidget", "Box", 0));
    connect(actionShapeBox_, SIGNAL(triggered()), this, SLOT(showBox()));

    actionShapeOpen_ = new QAction(this);
    actionShapeOpen_->setText(QApplication::translate("RenderingWidget", "Open...", 0));
    connect(actionShapeOpen_, SIGNAL(triggered()), this, SLOT(showLoadedModel()));
}

void RenderingWidget::resetCameraPosition()
{
    if (!renderingScene_) return;

    osgGA::CameraManipulator* cm = renderingScene_->getCameraManipulator();

    RenderingScene::fitCameraPosition(renderingScene_->getCamera(),
                                      osg::Vec3(0.0, -1.0, 0.0),
                                      osg::Vec3(0.0, 0.0, 1.0),
                                      renderingScene_->getScene());

    osg::Vec3d eye, center, up;
    renderingScene_->getCamera()->getViewMatrixAsLookAt(eye, center, up);
    center = scene_util::computeCenter(renderingScene_->getScene());
    cm->setHomePosition(eye, center, up);
    cm->home(0.0);
}

void RenderingWidget::showSphere()
{
    renderingScene_->getScene()->removeChildren(0, renderingScene_->getScene()->getNumChildren());

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), 1.0f)));
    renderingScene_->getScene()->addChild(geode);
}

void RenderingWidget::showCylinder()
{
    renderingScene_->getScene()->removeChildren(0, renderingScene_->getScene()->getNumChildren());
    
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(), 0.75f, 2.0f)));
    renderingScene_->getScene()->addChild(geode);
}

void RenderingWidget::showBox()
{
    renderingScene_->getScene()->removeChildren(0, renderingScene_->getScene()->getNumChildren());

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(), 1.5f)));
    renderingScene_->getScene()->addChild(geode);
}

void RenderingWidget::showLoadedModel()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Model File"), QString(),
                                                    tr("Obj Files (*.obj)"));
    
    if (fileName.isEmpty()) return;

    openModel(fileName);
    resetCameraPosition();
}

void RenderingWidget::resizeEvent(QResizeEvent* event)
{
    osgQt::GLWidget::resizeEvent(event);
    
    if (renderingScene_) {
        renderingScene_->updateView(event->size().width(), event->size().height());
    }
}

void RenderingWidget::keyPressEvent(QKeyEvent* event)
{
    osgQt::GLWidget::keyPressEvent(event);

    if (!renderingScene_) return;

    osgGA::CameraManipulator* cm = renderingScene_->getCameraManipulator();
    osgGA::TrackballManipulator* tm = dynamic_cast<osgGA::TrackballManipulator*>(cm);

    osg::Vec3 camPos, camCenter, camUp;
    renderingScene_->getCamera()->getViewMatrixAsLookAt(camPos, camCenter, camUp);
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

void RenderingWidget::mouseMoveEvent(QMouseEvent* event)
{
    osgQt::GLWidget::mouseMoveEvent(event);
    movedMouse_ = true;
}

void RenderingWidget::mousePressEvent(QMouseEvent* event)
{
    osgQt::GLWidget::mousePressEvent(event);
    movedMouse_ = false;
}

void RenderingWidget::dragEnterEvent(QDragEnterEvent* event)
{
    if (event->mimeData()->hasUrls()) {
        event->acceptProposedAction();
    }
}

void RenderingWidget::dropEvent(QDropEvent* event)
{
    QList<QUrl> urls = event->mimeData()->urls();
    QString fileName = urls.front().toLocalFile();
    openModel(fileName);
}

void RenderingWidget::wheelEvent(QWheelEvent* event)
{
    osgQt::GLWidget::wheelEvent(event);
    movedMouse_ = true;
}

void RenderingWidget::contextMenuEvent(QContextMenuEvent* event)
{
    osgQt::GLWidget::contextMenuEvent(event);

    if (movedMouse_) return;

    QMenu menu(this);
    menu.addAction(actionResetCamera_);
    
    QMenu* shapeMenu = menu.addMenu(tr("Shape"));
    shapeMenu->addAction(actionShapeSphere_);
    shapeMenu->addAction(actionShapeCylinder_);
    shapeMenu->addAction(actionShapeBox_);
    shapeMenu->addAction(actionShapeOpen_);

    menu.exec(event->globalPos());
}

void RenderingWidget::openModel(const QString& fileName)
{
    osg::Node* loadedModel = osgDB::readNodeFile(fileName.toLocal8Bit().data());
    if (!loadedModel) {
        QMessageBox::warning(this, tr("BSDF Viewer"),
                             tr("Failed to load \"") + fileName + "\"",
                             QMessageBox::Ok);
        return;
    }

    renderingScene_->getScene()->removeChildren(0, renderingScene_->getScene()->getNumChildren());

    osgUtil::Optimizer optimzer;
    optimzer.optimize(loadedModel, osgUtil::Optimizer::ALL_OPTIMIZATIONS);
    renderingScene_->getScene()->addChild(loadedModel);
}
