// =================================================================== //
// Copyright (C) 2019-2026 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef TABLE_SCENE_H
#define TABLE_SCENE_H

#include <memory>

#include <QtWidgets>

#include "MaterialData.h"

/*!
 * \class   TableScene
 * \brief   The TableScene class provides the scene of table view.
 */
class TableScene : public QGraphicsScene
{
    Q_OBJECT

public:
    explicit TableScene(QObject *parent = nullptr);

    void setMaterialData(MaterialData* materialData) { data_ = materialData; }

    void setBackSideShown(bool backSideShown) { backSideShown_ = backSideShown; }

    /*! Gets the value of a sample point for a item. */
    float getSampleValue(const lb::Spectrum& sp,
                         lb::ColorModel      colorModel,
                         const lb::Arrayf&   wavelengths);

    /*! Gets angle indices of lb::SampleSet at \a pos. */
    bool getIndex(const QPointF& pos, int* i0, int* i1, int* i2, int* i3);

    void initTable(int wavelengthIndex, float gamma, bool photometric);
    void updateTable();

    bool getInOutDir(const QPointF& pos, lb::Vec3* inDir, lb::Vec3* outDir);

signals:
    void mouseMoved(QPointF pos);
    void mouseClicked(QPointF pos);
    void mouseDoubleClicked(QPointF pos);

private:
    Q_DISABLE_COPY(TableScene)

    void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;
    void drawBackground(QPainter* painter, const QRectF& rect) override;

    void createBrdfTable();
    void createBrdfImage();
    void createBrdfAngleItems(const lb::SampleSet& ss);

    void createReflectanceTable();
    void createReflectanceDataItems(const lb::SampleSet2D& ss2);
    void createReflectanceAngleItems(const lb::SampleSet2D& ss2);

    void addAngleItem(const QColor& color,
                      float         angle,
                      qreal         posX,
                      qreal         posY,
                      qreal         lodThreshold = 25.0,
                      qreal         textLodThreshold = 25.0);

    std::unique_ptr<QImage> brdfImage_;

    MaterialData* data_;

    QColor backgroundColor_;

    int   wavelengthIndex_;
    float gamma_;
    bool  photometric_;
    bool  backSideShown_;
};

#endif // TABLE_SCENE_H