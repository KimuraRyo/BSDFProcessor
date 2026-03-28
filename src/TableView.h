// =================================================================== //
// Copyright (C) 2014-2026 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef TABLE_VIEW_H
#define TABLE_VIEW_H

#include <QtWidgets>

#include "MaterialData.h"
#include "TableScene.h"

/*!
 * \class   TableView
 * \brief   The TableView class provides the table view of sample points.
 */
class TableView : public QGraphicsView
{
    Q_OBJECT

public:
    explicit TableView(QWidget* parent = nullptr);

    void updateTable(int wavelengthIndex, float gamma, bool photometric);

    void setMaterialData(MaterialData* materialData);
    void setLogPlotAction(QAction* action) { actionLogPlot_ = action; }

    bool isBackSideShown() const { return backSideShown_; }
    void showBackSide(bool on);

signals:
    void inOutDirPicked(const lb::Vec3& inDir, const lb::Vec3& outDir);
    void clearPickedValue();

public slots:
    void fitView(qreal scaleFactor = 1.0);

private slots:
    void changeBackSideVisibility();
    void showToolTip(const QPointF& pos);
    void updateInOutDir(const QPointF& pos);

private:
    Q_DISABLE_COPY(TableView)

    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void contextMenuEvent(QContextMenuEvent* event) override;
    void showEvent(QShowEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

    TableScene* tableScene_;

    QAction* actionFitView_;
    QAction* actionLogPlot_;
    QAction* actionShowBackSide_;

    MaterialData* data_;

    bool backSideShown_;
    bool fittingNeeded_;
};

#endif // TABLE_VIEW_H