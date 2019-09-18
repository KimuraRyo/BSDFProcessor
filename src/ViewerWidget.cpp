// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "ViewerWidget.h"

#include <QtWidgets/QVBoxLayout>

#include <libbsdf/Brdf/Brdf.h>
#include <libbsdf/Brdf/SampleSet2D.h>

#include "SceneUtil.h"

ViewerWidget::ViewerWidget(QWidget* parent) : QWidget(parent)
{
    setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);
    //setThreadingModel(osgViewer::CompositeViewer::CullThreadPerCameraDrawThreadPerContext);

    // Disable the default setting of viewer.done() by pressing Escape.
    setKeyEventSetsDone(0);

    setRunFrameScheme(osgViewer::ViewerBase::ON_DEMAND);

    connect(&timer_, SIGNAL(timeout()), this, SLOT(update()));
    timer_.start(10);
}

void ViewerWidget::addViewWidget(osgQt::GLWidget* qglWidget)
{
    osgViewer::View* view = new osgViewer::View;
    addView(view);
    initializeViewer(qglWidget, view);

#if defined(DEVELOPMENT)
    setupEventHandler(view);
#endif

    QVBoxLayout* vLayout = new QVBoxLayout();
    vLayout->addWidget(qglWidget);
    vLayout->setContentsMargins(1, 1, 1, 1);
    setLayout(vLayout);

    scene_util::displayGlInformation(view->getCamera()->getGraphicsContext());
}

QGLFormat ViewerWidget::createQGLFormat(int x, int y, int w, int h,
                                        const std::string& name, bool windowDecoration)
{
    osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
    osg::GraphicsContext::Traits* traits = new osg::GraphicsContext::Traits;
    traits->windowName = name;
    traits->windowDecoration = windowDecoration;
    traits->x = x;
    traits->y = y;
    traits->width = w;
    traits->height = h;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;
    //traits->depth = 32;
    traits->alpha = ds->getMinimumNumAlphaBits();
    traits->stencil = ds->getMinimumNumStencilBits();
    traits->sampleBuffers = ds->getMultiSamples();
    traits->samples = ds->getNumMultiSamples();
    traits->supportsResize = true;

    return osgQt::GraphicsWindowQt::traits2qglFormat(traits);
}

void ViewerWidget::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    if (checkNeedToDoFrame()) {
        frame();
    }
}

void ViewerWidget::initializeViewer(osgQt::GLWidget* qglWidget, osgViewer::View* view)
{
    graphicsWindow_ = new osgQt::GraphicsWindowQt(qglWidget);

    osg::Camera* camera = view->getCamera();
    camera->setGraphicsContext(graphicsWindow_.get());
    camera->setClearColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));

    double aspectRatio = static_cast<double>(qglWidget->width())
                       / static_cast<double>(qglWidget->height());
    camera->setProjectionMatrixAsPerspective(30.0, aspectRatio, 0.00001, 100000.0);
    camera->setViewport(0, 0, qglWidget->width(), qglWidget->height());

    osgGA::TrackballManipulator* trackball = new osgGA::TrackballManipulator;
    trackball->setHomePosition(osg::Vec3d(0.0, 10.0, 0.0), osg::Vec3d(0.0, 0.0, 0.0), osg::Vec3d(0.0, 0.0, 1.0));
    trackball->home(0.0);
    trackball->setMinimumDistance(0.0001);
    trackball->setAllowThrow(false);
    view->setCameraManipulator(trackball);
}

void ViewerWidget::setupEventHandler(osgViewer::View* view)
{
    // backface culling: 'b'
    // toggle lighting: 'l'
    // texturing: 't'
    // polygon mode: 'w'
    view->addEventHandler(new osgGA::StateSetManipulator(view->getCamera()->getOrCreateStateSet()));

    // 's'
    view->addEventHandler(new osgViewer::StatsHandler);

    // 'm'
    view->addEventHandler(new osgViewer::ThreadingHandler);
}
