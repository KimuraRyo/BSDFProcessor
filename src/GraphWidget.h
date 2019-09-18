// =================================================================== //
// Copyright (C) 2014-2018 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef GRAPH_WIDGET_H
#define GRAPH_WIDGET_H

#include <QOpenGLContext>

#include <QtGui/QResizeEvent>
#include <QtGui/QKeyEvent>
#include <QtGui/QMouseEvent>

#include <osgQt/GraphicsWindowQt>

class GraphScene;

/*!
 * \class   GraphWidget
 * \brief   The GraphWidget class provides QGLWidget for 3D graph.
 */
class GraphWidget : public osgQt::GLWidget
{
    Q_OBJECT

public:
    explicit GraphWidget(const QGLFormat&   format,
                         QWidget*           parent = 0,
                         const QGLWidget*   shareWidget = 0,
                         Qt::WindowFlags    f = 0,
                         bool               forwardKeyEvents = false);

    void setGraphScene(GraphScene* scene) { graphScene_ = scene; }

    QAction* getLogPlotAction() { return actionLogPlot_; }

signals:
    void fileDropped(const QString& fileName);
    void picked(const osg::Vec3& position);
    void clearPickedValue();
    void viewFront();
    void logPlotToggled(bool on);

private slots:
    void resetCameraPosition() { emit viewFront(); }
    void copyCameraSettings();
    void pasteCameraSettings();
    void toggleLogPlot(bool on) { emit logPlotToggled(on); }

private:
    Q_DISABLE_COPY(GraphWidget)

    void resizeEvent(QResizeEvent* event);

    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void dragEnterEvent(QDragEnterEvent* event);
    void dropEvent(QDropEvent* event);
    void wheelEvent(QWheelEvent* event);
    void contextMenuEvent(QContextMenuEvent* event);

    void showContextMenu(const QPoint& pos);

    static bool getParameters(const QStringList& paramList, const QString& name, osg::Vec3d* params);

    GraphScene* graphScene_;

    bool mouseMoved_;

    QAction* actionResetCamera_;
    QAction* actionCopyCameraSettings_;
    QAction* actionPasteCameraSettings_;
    QAction* actionLogPlot_;
};

#endif // GRAPH_WIDGET_H
