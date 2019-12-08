// =================================================================== //
// Copyright (C) 2014-2019 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef GRAPH_WIDGET_H
#define GRAPH_WIDGET_H

#include "GraphScene.h"
#include "OsgQWidget.h"

/*!
 * \class   GraphWidget
 * \brief   The GraphWidget class provides a OSG widget for 3D graph.
 */
class GraphWidget : public OsgQWidget
{
    Q_OBJECT

public:
    explicit GraphWidget(QWidget* parent = nullptr, Qt::WindowFlags f = 0);

    void setGraphScene(GraphScene* scene);

    void updateView();

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

    void initializeGL() override;
    void resizeGL(int w, int h) override;

    void keyPressEvent(QKeyEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void dragEnterEvent(QDragEnterEvent* event) override;
    void dropEvent(QDropEvent* event) override;
    void contextMenuEvent(QContextMenuEvent* event) override;

    void showContextMenu(const QPoint& pos);

    static bool getParameters(const QStringList& paramList, const QString& name, osg::Vec3d* params);

    GraphScene* graphScene_;

    QAction* actionResetCamera_;
    QAction* actionCopyCameraSettings_;
    QAction* actionPasteCameraSettings_;
    QAction* actionLogPlot_;
};

#endif // GRAPH_WIDGET_H
