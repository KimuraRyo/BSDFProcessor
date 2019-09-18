// =================================================================== //
// Copyright (C) 2014-2018 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef VIEWER_WIDGET_H
#define VIEWER_WIDGET_H

#include <QOpenGLContext>

#include <QtCore/QTimer>
#include <QtWidgets/QWidget>

#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgQt/GraphicsWindowQt>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>

/*!
 * \class   ViewerWidget.h
 * \brief   The ViewerWidget class provides QWidget and osgViewer::CompositeViewer.
 */
class ViewerWidget : public QWidget, public osgViewer::CompositeViewer
{
public:
    explicit ViewerWidget(QWidget* parent);

    void addViewWidget(osgQt::GLWidget* qglWidget);

    static QGLFormat createQGLFormat(int x = 0, int y = 0, int w = 1024, int h = 1024,
                                     const std::string& name = "", bool windowDecoration = false);

private:
    Q_DISABLE_COPY(ViewerWidget)

    void paintEvent(QPaintEvent* event);

    void initializeViewer(osgQt::GLWidget* qglWidget, osgViewer::View* view);

    /*! Sets up event handlers in the view. */
    static void setupEventHandler(osgViewer::View* view);

    QTimer timer_;

    osg::ref_ptr<osgQt::GraphicsWindowQt> graphicsWindow_; /*!< Graphics context. */
};

#endif // VIEWER_WIDGET_H
