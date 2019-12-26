// =================================================================== //
// Copyright (C) 2014-2019 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef TABLE_VIEW_H
#define TABLE_VIEW_H

#include <QtWidgets/QGraphicsView>

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

    void createTable(int wavelengthIndex, float gamma = 1.0f, bool photometric = false);

    void setMaterialData(MaterialData* materialData) { data_ = materialData; }

signals:
    void inOutDirPicked(const lb::Vec3& inDir, const lb::Vec3& outDir);
    void inDirPicked(const lb::Vec3& inDir);

public slots:
    void fitView(qreal scaleFactor = 1.0);

private slots:
    void changeBackSideVisibility();
    void showToolTip(const QPointF& pos);
    void updateInOutDirection(const QPointF& pos);
    void updateInDirection(const QPointF& pos);

private:
    Q_DISABLE_COPY(TableView)

    void createBrdfTable(int wavelengthIndex);
    void createBrdfDataItems(int wavelengthIndex);
    void createBrdfDataPixmapItem(int wavelengthIndex);
    void createBrdfAngleItems(const lb::SampleSet& ss);

    void createReflectanceTable(int wavelengthIndex);
    void createReflectanceDataItems(const lb::SampleSet2D& ss2, int wavelengthIndex);
    void createReflectanceAngleItems(const lb::SampleSet2D& ss2);

    /*! Gets the value of a sample point for a item. */
    float getSampleValue(const lb::Spectrum&    sp,
                         lb::ColorModel         colorModel,
                         const lb::Arrayf&      wavelengths,
                         int                    wavelengthIndex);

    /*! Gets angle indices of lb::SampleSet at \a pos. */
    bool getIndex(const QPointF& pos, int* i0, int* i1, int* i2, int* i3);

    bool getInOutDir(const QPointF& pos, lb::Vec3* inDir, lb::Vec3* outDir);

    void addAngleItem(const QColor& color, float angle,
                      qreal posX, qreal posY,
                      qreal lodThreshold = 25.0, qreal textLodThreshold = 25.0);

    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void contextMenuEvent(QContextMenuEvent* event) override;
    void showEvent(QShowEvent* event) override;

    TableScene* graphicsScene_;

    QAction* actionFitView_;
    QAction* actionShowBackSide_;

    MaterialData* data_;

    int     wavelengthIndex_;
    float   gamma_;
    bool    photometric_;
    bool    backSideShown_;

    bool fittingNeeded_;
};

#endif // TABLE_VIEW_H
