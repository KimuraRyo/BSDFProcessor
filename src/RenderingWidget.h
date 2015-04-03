// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef RENDERING_WIDGET_H
#define RENDERING_WIDGET_H

#include <QtGui/QResizeEvent>
#include <QtGui/QKeyEvent>
#include <QtGui/QMouseEvent>

#include <osgQt/GraphicsWindowQt>

class RenderingScene;

/*!
 * \class   RenderingWidget
 * \brief   The RenderingWidget class provides QGLWidget for a rendering view.
 */
class RenderingWidget : public osgQt::GLWidget
{
    Q_OBJECT

public:
    explicit RenderingWidget(const QGLFormat&   format,
                             QWidget*           parent = 0,
                             const QGLWidget*   shareWidget = 0,
                             Qt::WindowFlags    f = 0,
                             bool               forwardKeyEvents = false);

    void setRenderingScene(RenderingScene* scene)
    {
        renderingScene_ = scene;
        showSphere();
        resetCameraPosition();
    }

private slots:
    void resetCameraPosition();
    void showSphere();
    void showCylinder();
    void showBox();
    void showLoadedModel();

private:
    Q_DISABLE_COPY(RenderingWidget)

    void resizeEvent(QResizeEvent* event);

    void keyPressEvent(QKeyEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void dragEnterEvent(QDragEnterEvent* event);
    void dropEvent(QDropEvent* event);
    void wheelEvent(QWheelEvent* event);
    void contextMenuEvent(QContextMenuEvent* event);

    void openModel(const QString& fileName);

    RenderingScene* renderingScene_;

    bool movedMouse_;

    QAction* actionResetCamera_;
    QAction* actionShapeSphere_;
    QAction* actionShapeCylinder_;
    QAction* actionShapeBox_;
    QAction* actionShapeOpen_;
};

#endif // RENDERING_WIDGET_H
