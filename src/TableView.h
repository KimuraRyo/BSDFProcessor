// =================================================================== //
// Copyright (C) 2014-2018 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef TABLE_VIEW_H
#define TABLE_VIEW_H

#include <QtWidgets/QGraphicsView>

#include "MaterialData.h"

namespace lb
{
    class SampleSet;
}

/*!
 * \class   TableView
 * \brief   The TableView class provides the table view of sample points.
 */
class TableView : public QGraphicsView
{
    Q_OBJECT

public:
    explicit TableView(QWidget* parent = 0);

    void createTable(int wavelengthIndex, float gamma = 1.0f, bool photometric = false);

    void setMaterialData(MaterialData* data) { data_ = data; }

public slots:
    void fitView(qreal scaleFactor = 1.0)
    {
        fitInView(scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
        scale(scaleFactor, scaleFactor);
    }

private:
    Q_DISABLE_COPY(TableView)

    void createBrdfTable(int wavelengthIndex);
    void createBrdfDataItems(int wavelengthIndex);
    void createBrdfDataPixmapItem(int wavelengthIndex);

    void createBrdfAngleItems(const lb::SampleSet* ss);

    void createReflectanceTable(int wavelengthIndex);

    /*! Gets the value of a sample point for a item. */
    float getSampleValue(const lb::Spectrum&    sp,
                         lb::ColorModel         colorModel,
                         const lb::Arrayf&      wavelengths,
                         int                    wavelengthIndex);

    void addAngleItem(const QColor& color, float angle,
                      qreal posX, qreal posY,
                      qreal lodThreshold = 25.0, qreal textLodThreshold = 25.0);

    void paintEvent(QPaintEvent* event);
    void wheelEvent(QWheelEvent* event);
    void contextMenuEvent(QContextMenuEvent* event);

    QGraphicsScene* graphicsScene_;

    QAction* actionFitView_;
    QAction* actionShowBackSide_;

    MaterialData* data_;

    int     wavelengthIndex_;
    float   gamma_;
    bool    photometric_;
    bool    backSideShown_;

private slots:
    void changeBackSideVisibility()
    {
        backSideShown_ = !backSideShown_;
        createTable(wavelengthIndex_, gamma_, photometric_);
    }
};

#endif // TABLE_VIEW_H
