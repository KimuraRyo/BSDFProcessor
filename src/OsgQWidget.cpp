// =================================================================== //
// Copyright (C) 2019 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "OsgQWidget.h"

//#include <osgViewer/ViewerEventHandlers>

OsgQWidget::OsgQWidget(QWidget*         parent,
                       Qt::WindowFlags  f)
                       : QOpenGLWidget(parent, f),
                         viewer_(new osgViewer::Viewer),
                         mouseMoved_(false),
                         osgFboIdInitialized_(false)
{
    viewer_->setThreadingModel(osgViewer::Viewer::SingleThreaded);

    //viewer_->addEventHandler(new osgViewer::StatsHandler);
}

void OsgQWidget::initializeGL()
{
    // An OpenGL context is set up in initializeGL(), not the constructor. Because the OpenGL context of QOpenGLWidget in QDockWidget is
    // reinitialized while docking/undoking.
    int pr = static_cast<int>(getPixelRatio());
    graphicsWindow_ = viewer_->setUpViewerAsEmbeddedInWindow(x() * pr, y() * pr, width() * pr, height() * pr);
    viewer_->getCamera()->setGraphicsContext(graphicsWindow_);
    viewer_->realize();

    osgFboIdInitialized_ = false;
}

void OsgQWidget::paintGL()
{
    if (!osgFboIdInitialized_) {
        // Configure the FBO ID of OSG for QOpenGLWidget.
        GLuint qtFboId = defaultFramebufferObject();
        viewer_->getCamera()->getGraphicsContext()->setDefaultFboId(qtFboId);

        osgFboIdInitialized_ = true;
    }

    viewer_->frame();
}

void OsgQWidget::resizeGL(int w, int h)
{
    int pr = static_cast<int>(getPixelRatio());

    graphicsWindow_->getEventQueue()->windowResize(x() * pr, y() * pr, w * pr, h * pr);
    graphicsWindow_->resized(x() * pr, y() * pr, w * pr, h * pr);

    viewer_->getCamera()->setViewport(0, 0, w * pr, h * pr);
}

void OsgQWidget::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    makeCurrent();
    paintGL();
    doneCurrent();
}

void OsgQWidget::keyPressEvent(QKeyEvent* event)
{
    const char* keyData = event->text().toLocal8Bit().data();
    graphicsWindow_->getEventQueue()->keyPress(osgGA::GUIEventAdapter::KeySymbol(*keyData));

    update();
}

void OsgQWidget::keyReleaseEvent(QKeyEvent* event)
{
    const char* keyData = event->text().toLocal8Bit().data();
    graphicsWindow_->getEventQueue()->keyRelease(osgGA::GUIEventAdapter::KeySymbol(*keyData));

    update();
}

void OsgQWidget::mouseMoveEvent(QMouseEvent* event)
{
    mouseMoved_ = true;

    osg::Vec2f pos = getPosition(*event);
    graphicsWindow_->getEventQueue()->mouseMotion(pos.x(), pos.y());

    update();
}

void OsgQWidget::mousePressEvent(QMouseEvent* event)
{
    mouseMoved_ = false;

    osg::Vec2f pos = getPosition(*event);
    int button = getOsgMouseButton(*event);
    graphicsWindow_->getEventQueue()->mouseButtonPress(pos.x(), pos.y(), button);

    update();
}

void OsgQWidget::mouseReleaseEvent(QMouseEvent* event)
{
    osg::Vec2f pos = getPosition(*event);
    int button = getOsgMouseButton(*event);
    graphicsWindow_->getEventQueue()->mouseButtonRelease(pos.x(), pos.y(), button);

    update();
}

void OsgQWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
    osg::Vec2f pos = getPosition(*event);
    int button = getOsgMouseButton(*event);
    graphicsWindow_->getEventQueue()->mouseDoubleButtonPress(pos.x(), pos.y(), button);

    update();
}

void OsgQWidget::wheelEvent(QWheelEvent* event)
{
    mouseMoved_ = true;

    int delta = event->delta();
    osgGA::GUIEventAdapter::ScrollingMotion motion = (delta > 0) ? osgGA::GUIEventAdapter::SCROLL_UP
                                                                 : osgGA::GUIEventAdapter::SCROLL_DOWN;
    graphicsWindow_->getEventQueue()->mouseScroll(motion);

    event->accept();
    update();
}

osg::Vec2f OsgQWidget::getPosition(const QMouseEvent& event) const
{
    return osg::Vec2f(event.x(), event.y()) * getPixelRatio();
}

int OsgQWidget::getOsgMouseButton(const QMouseEvent& event)
{
    unsigned int button = 0;

    switch (event.button()) {
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

    return button;
}
