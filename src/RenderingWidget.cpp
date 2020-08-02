// =================================================================== //
// Copyright (C) 2014-2020 Kimura Ryo                                  //
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

#include "SceneUtil.h"

RenderingWidget::RenderingWidget(QWidget*           parent,
                                 Qt::WindowFlags    f)
                                 : OsgQWidget(parent, f),
                                   renderingScene_(nullptr),
                                   skipRequested_(false)
{
    osg::Camera* camera = new osg::Camera;
    QSize viewSize = size() * getPixelRatio();
    camera->setViewport(0, 0, viewSize.width(), viewSize.height());
    camera->setClearColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));
    double aspectRatio = static_cast<double>(width()) / height();
    camera->setProjectionMatrixAsPerspective(30.0, aspectRatio, 0.00001, 100000.0);
    viewer_->setCamera(camera);

    osgGA::TrackballManipulator* trackball = new osgGA::TrackballManipulator;
    trackball->setHomePosition(osg::Vec3d(0.0, 10.0, 0.0), osg::Vec3d(0.0, 0.0, 0.0), osg::Vec3d(0.0, 0.0, 1.0));
    trackball->home(0.0);
    trackball->setMinimumDistance(0.0001);
    trackball->setAllowThrow(false);
    viewer_->setCameraManipulator(trackball);

    actionResetCamera_ = new QAction(this);
    actionResetCamera_->setText("Reset camera position");
    connect(actionResetCamera_, SIGNAL(triggered()), this, SLOT(resetCameraPosition()));

    actionShapeSphere_ = new QAction(this);
    actionShapeSphere_->setText("Sphere");
    connect(actionShapeSphere_, SIGNAL(triggered()), this, SLOT(showSphere()));

    actionShapeCylinder_ = new QAction(this);
    actionShapeCylinder_->setText("Cylinder");
    connect(actionShapeCylinder_, SIGNAL(triggered()), this, SLOT(showCylinder()));

    actionShapeBox_ = new QAction(this);
    actionShapeBox_->setText("Box");
    connect(actionShapeBox_, SIGNAL(triggered()), this, SLOT(showBox()));

    actionShapeOpen_ = new QAction(this);
    actionShapeOpen_->setText("Open...");
    connect(actionShapeOpen_, SIGNAL(triggered()), this, SLOT(showLoadedModel()));
}

void RenderingWidget::setRenderingScene(RenderingScene* scene)
{
    renderingScene_ = scene;
    viewer_->setSceneData(renderingScene_->getRoot());
    showSphere();
    resetCameraPosition();
}

void RenderingWidget::updateView()
{
    QSize viewSize = size() * getPixelRatio();
    renderingScene_->updateView(viewSize.width(), viewSize.height());
    update();
}

void RenderingWidget::resetCameraPosition()
{
    if (!renderingScene_) return;

    osg::Group* scene = renderingScene_->getScene();
    RenderingScene::fitCameraPosition(viewer_->getCamera(),
                                      osg::Vec3(0.0, -1.0, 0.0),
                                      osg::Vec3(0.0, 0.0, 1.0),
                                      scene);

    osg::Vec3d eye, center, up;
    viewer_->getCamera()->getViewMatrixAsLookAt(eye, center, up);
    center = scene_util::computeCenter(scene);

    osgGA::CameraManipulator* cm = viewer_->getCameraManipulator();
    cm->setHomePosition(eye, center, up);
    cm->home(0.0);
}

void RenderingWidget::showSphere()
{
    osg::Group* scene = renderingScene_->getScene();
    scene->removeChildren(0, scene->getNumChildren());

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), 1.0f)));
    scene->addChild(geode);
}

void RenderingWidget::showCylinder()
{
    osg::Group* scene = renderingScene_->getScene();
    scene->removeChildren(0, scene->getNumChildren());

    osg::Geode* geode = new osg::Geode;
    osg::Cylinder* cylinder = new osg::Cylinder(osg::Vec3(), 1.0f, 2.0f);
    osg::Quat rotQuat(lb::PI_D / 2.0, osg::Vec3(0.0, 1.0, 0.0));
    cylinder->setRotation(rotQuat);
    geode->addDrawable(new osg::ShapeDrawable(cylinder));
    scene->addChild(geode);
}

void RenderingWidget::showBox()
{
    osg::Group* scene = renderingScene_->getScene();
    scene->removeChildren(0, scene->getNumChildren());

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(), 1.5f)));
    scene->addChild(geode);
}

void RenderingWidget::showLoadedModel()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open Model File", QString(), "Wavefront OBJ (*.obj)");
    if (fileName.isEmpty()) return;

    openModel(fileName);
    resetCameraPosition();
}

void RenderingWidget::paintGL()
{
    if (skipRequested_) {
        skipRequested_ = false;
        return;
    }

    if (!osgFboIdInitialized_) {
        // Configure the FBO ID of OSG for QOpenGLWidget.
        GLuint qtFboId = defaultFramebufferObject();
        viewer_->getCamera()->getGraphicsContext()->setDefaultFboId(qtFboId);

        osgFboIdInitialized_ = true;
    }
 
    viewer_->frame();
}

void RenderingWidget::resizeGL(int w, int h)
{
    OsgQWidget::resizeGL(w, h);

    if (renderingScene_) {
        QSize viewSize = QSize(w, h) * getPixelRatio();
        renderingScene_->updateView(viewSize.width(), viewSize.height());
    }
}

void RenderingWidget::resizeEvent(QResizeEvent *event)
{
    OsgQWidget::resizeEvent(event);

    // This is a workaround for the instability of resizing. The OpenGL context of QOpenGLWidget
    // in QDockWidget is rarely broken while resizing.
    initializeGL();
}

void RenderingWidget::keyPressEvent(QKeyEvent* event)
{
    OsgQWidget::keyPressEvent(event);

    if (!renderingScene_) return;

    auto tm = dynamic_cast<osgGA::TrackballManipulator*>(viewer_->getCameraManipulator());
    if (!tm) return;

    osg::Vec3 camPos, camCenter, camUp;
    viewer_->getCamera()->getViewMatrixAsLookAt(camPos, camCenter, camUp);
    osg::Vec3 camDir = camPos - camCenter;
    camDir.normalize();

    switch (event->key()) {
        case Qt::Key_Up: {
            if (!tm) break;

            osg::Vec3 rotAxis = camDir ^ camUp;
            osg::Quat rotQuat(lb::PI_D / 180.0, rotAxis);
            tm->setRotation(tm->getRotation() * rotQuat);

            break;
        }
        case Qt::Key_Down: {
            if (!tm) break;

            osg::Vec3 rotAxis = camDir ^ camUp;
            osg::Quat rotQuat(-lb::PI_D / 180.0, rotAxis);
            tm->setRotation(tm->getRotation() * rotQuat);

            break;
        }
        case Qt::Key_Left: {
            if (!tm) break;

            osg::Vec3 rotAxis(0.0, 0.0, 1.0);
            osg::Quat rotQuat(-lb::PI_D / 180.0, rotAxis);
            tm->setRotation(tm->getRotation() * rotQuat);

            break;
        }
        case Qt::Key_Right: {
            if (!tm) break;

            osg::Vec3 rotAxis(0.0, 0.0, 1.0);
            osg::Quat rotQuat(lb::PI_D / 180.0, rotAxis);
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

void RenderingWidget::mousePressEvent(QMouseEvent* event)
{
    OsgQWidget::mousePressEvent(event);

    skipRequested_ = true;
}

void RenderingWidget::mouseReleaseEvent(QMouseEvent* event)
{
    OsgQWidget::mouseReleaseEvent(event);

    if (event->button() == Qt::LeftButton && !mouseMoved_) {
        skipRequested_ = true;

        if (!renderingScene_) return;

        QPoint pixelPos = event->pos() * getPixelRatio();
        lb::Vec3 inDir  = renderingScene_->getInDir(pixelPos.x(), pixelPos.y());
        lb::Vec3 outDir = renderingScene_->getOutDir(pixelPos.x(), pixelPos.y());
        if (inDir.isZero() || outDir.isZero()) {
            emit clearPickedValue();
            return;
        }

        const lb::Brdf* brdf = renderingScene_->getBrdf();
        const lb::SampleSet2D* ss2 = renderingScene_->getReflectance();

        if (brdf) {
            if (brdf->getSampleSet()->isIsotropic()) {
                float inTheta, outTheta, outPhi;
                lb::SphericalCoordinateSystem::fromXyz(inDir, outDir, &inTheta, &outTheta, &outPhi);
                lb::SphericalCoordinateSystem::toXyz(inTheta, 0.0f, outTheta, outPhi, &inDir, &outDir);
            }

            lb::Spectrum sp = brdf->getSpectrum(inDir, outDir);

            lbDebug << "[RenderingWidget::mouseReleaseEvent] inDir: "    << inDir.format(lb::LB_EIGEN_IO_FMT);
            lbDebug << "[RenderingWidget::mouseReleaseEvent] outDir: "   << outDir.format(lb::LB_EIGEN_IO_FMT);
            lbDebug << "[RenderingWidget::mouseReleaseEvent] Spectrum: " << sp.format(lb::LB_EIGEN_IO_FMT);
        }
        else if (ss2) {
            if (ss2->isIsotropic()) {
                float theta, phi;
                lb::SphericalCoordinateSystem::fromXyz(outDir, &theta, &phi);
                outDir = lb::SphericalCoordinateSystem::toXyz(theta, lb::PI_F);
            }

            lb::DataType dataType = renderingScene_->getDataType();
            switch (dataType) {
                case lb::SPECULAR_REFLECTANCE_DATA:
                    inDir = lb::reflect(outDir, lb::Vec3(0.0, 0.0, 1.0));
                    break;
                case lb::SPECULAR_TRANSMITTANCE_DATA:
                    inDir = -outDir;
                    break;
                default:
                    lbWarn << "[RenderingWidget::mouseReleaseEvent] Invalid data type: " << dataType;
                    break;
            }
        }

        emit inOutDirPicked(inDir, outDir);
    }
#ifdef __APPLE__
    else if (event->button() == Qt::RightButton && !mouseMoved_) {
        showContextMenu(event->globalPos());
    }
#endif
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

void RenderingWidget::contextMenuEvent(QContextMenuEvent* event)
{
#ifndef __APPLE__
    if (mouseMoved_) return;

    showContextMenu(event->globalPos());
#endif

    update();
}

void RenderingWidget::showContextMenu(const QPoint& pos)
{
    QMenu menu(this);
    menu.addAction(actionResetCamera_);

    QMenu* shapeMenu = menu.addMenu("Shape");
    shapeMenu->addAction(actionShapeSphere_);
    shapeMenu->addAction(actionShapeCylinder_);
    shapeMenu->addAction(actionShapeBox_);
    shapeMenu->addAction(actionShapeOpen_);

    menu.exec(pos);
}

void RenderingWidget::openModel(const QString& fileName)
{
    osg::Node* loadedModel = osgDB::readNodeFile(fileName.toLocal8Bit().data());
    if (!loadedModel) {
        QMessageBox::warning(this, qApp->applicationName(), "Failed to load \"" + fileName + "\"");
        return;
    }

    osg::Group* scene = renderingScene_->getScene();
    scene->removeChildren(0, scene->getNumChildren());

    osgUtil::Optimizer optimzer;
    optimzer.optimize(loadedModel, osgUtil::Optimizer::ALL_OPTIMIZATIONS);
    scene->addChild(loadedModel);
}
