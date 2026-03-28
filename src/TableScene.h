// =================================================================== //
// Copyright (C) 2019-2026 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef TABLE_SCENE_H
#define TABLE_SCENE_H

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

    void setWavelengthIndex(int wavelengthIndex) { wavelengthIndex_ = wavelengthIndex; }
    void setGamma(float gamma) { gamma_ = gamma; }
    void setPhotometric(bool photometric) { photometric_ = photometric; }
    void setBackSideShown(bool backSideShown) { backSideShown_ = backSideShown; }

    /*! Gets the value of a sample point for a item. */
    float getSampleValue(const lb::Spectrum& sp,
                         lb::ColorModel      colorModel,
                         const lb::Arrayf&   wavelengths,
                         int                 wavelengthIndex);

    /*! Gets angle indices of lb::SampleSet at \a pos. */
    bool getIndex(const QPointF& pos, int* i0, int* i1, int* i2, int* i3);



    void setImage(const QImage& img)
    {
        image_ = img;
        setSceneRect(QRectF(QPointF(0, 0), QSizeF(image_.size())));
        update();
    }


signals:
    void mouseMoved(QPointF pos);
    void mouseClicked(QPointF pos);
    void mouseDoubleClicked(QPointF pos);

private:
    Q_DISABLE_COPY(TableScene)

    void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;
    void drawBackground(QPainter* painter, const QRectF& rect) override;

    QImage        image_;
    MaterialData* data_;

    QColor backgroundColor_;

    int   wavelengthIndex_;
    float gamma_;
    bool  photometric_;
    bool  backSideShown_;
};

#endif // TABLE_SCENE_H