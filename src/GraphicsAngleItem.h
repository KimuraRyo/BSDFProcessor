// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef GRAPHICS_ANGLE_ITEM_H
#define GRAPHICS_ANGLE_ITEM_H

#include <QtGui/QColor>
#include <QtWidgets/QGraphicsItem>

/*!
 * \class   GraphicsAngleItem
 * \brief   The GraphicsAngleItem class provides the angle item in a table view.
 */
class GraphicsAngleItem : public QGraphicsItem
{
public:
    GraphicsAngleItem(const QColor& color, float value);

    void setLodThreshold(qreal threshold) { lodThreshold_ = threshold; }

    void setTextLodThreshold(qreal threshold) { textLodThreshold_ = threshold; }

    QRectF boundingRect() const;
    QPainterPath shape() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem* option, QWidget* widget);

private:
    QColor color_;

    float value_;

    qreal width_;
    qreal height_;

    qreal lodThreshold_;
    qreal textLodThreshold_;
};

#endif // GRAPHICS_ANGLE_ITEM_H
