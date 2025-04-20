// =================================================================== //
// Copyright (C) 2014-2025 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef RENDERING_WIDGET_H
#define RENDERING_WIDGET_H

#include "RenderingScene.h"
#include "OsgQWidget.h"

/*!
 * \class   RenderingWidget
 * \brief   The RenderingWidget class provides a OSG widget for a rendering view.
 */
class RenderingWidget : public OsgQWidget
{
    Q_OBJECT

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit RenderingWidget(QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());

    void setRenderingScene(RenderingScene* scene);

    void updateView();

signals:
    void inOutDirPicked(const lb::Vec3& inDir, const lb::Vec3& outDir);
    void clearPickedValue();

private slots:
    void resetCameraPosition();
    void showSphere();
    void showCylinder();
    void showBox();
    void showLoadedModel();

private:
    Q_DISABLE_COPY(RenderingWidget)

    void paintGL() override;
    void resizeGL(int w, int h) override;

    void resizeEvent(QResizeEvent *event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void dragEnterEvent(QDragEnterEvent* event) override;
    void dropEvent(QDropEvent* event) override;
    void contextMenuEvent(QContextMenuEvent* event) override;

    void showContextMenu(const QPoint& pos);

    void openModel(const QString& fileName);

    RenderingScene* renderingScene_;

    bool skipRequested_;

    QAction* actionResetCamera_;
    QAction* actionShapeSphere_;
    QAction* actionShapeCylinder_;
    QAction* actionShapeBox_;
    QAction* actionShapeOpen_;
};

#endif // RENDERING_WIDGET_H
