// =================================================================== //
// Copyright (C) 2019-2025 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef OSG_QWIDGET_H
#define OSG_QWIDGET_H

#ifdef __APPLE__
#define __glext_h_
#include <QtGui/qopengl.h>
#undef __glext_h_
#include <QtGui/qopenglext.h>
#endif

#include <QtGui/QKeyEvent>
#include <QtGui/QMouseEvent>
#include <QOpenGLWidget>

#include <osgViewer/Viewer>

/*!
 * \class   OsgQWidget
 * \brief   The OsgQWidget class provides QOpenGLWidget for OpenSceneGraph.
 */
class OsgQWidget : public QOpenGLWidget
{
public:
    explicit OsgQWidget(QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());

    osgViewer::Viewer* getViewer() { return viewer_; }

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

    void paintEvent(QPaintEvent* event) override;

    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

    double getPixelRatio() const { return devicePixelRatioF(); }

    /*! Gets the position taking into account the device pixel ratio. */
    osg::Vec2f getPosition(const QMouseEvent& event) const;

    bool mouseMoved_;

    osg::ref_ptr<osgViewer::Viewer> viewer_;
    osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> graphicsWindow_;

    bool osgFboIdInitialized_;

private:
    Q_DISABLE_COPY(OsgQWidget)

    static int getOsgMouseButton(const QMouseEvent& event);
};

#endif // OSG_QWIDGET_H
