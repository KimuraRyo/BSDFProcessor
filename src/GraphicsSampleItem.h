// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef GRAPHICS_SAMPLE_ITEM_H
#define GRAPHICS_SAMPLE_ITEM_H

#include <QtGui/QColor>
#include <QtWidgets/QGraphicsItem>

/*!
 * \class   GraphicsSampleItem
 * \brief   The GraphicsSampleItem class provides the sample item in a table view.
 */
class GraphicsSampleItem : public QGraphicsItem
{
public:
    GraphicsSampleItem(const QColor& color, float value);

    QRectF boundingRect() const;
    QPainterPath shape() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem* item, QWidget* widget);

    bool isWhitish() { return whitish_; }

private:
    QColor color_;
    QColor textColor_;
    QColor gridColor_;

    float value_;

    bool whitish_;

    qreal width_;
    qreal height_;
};

#endif // GRAPHICS_SAMPLE_ITEM_H
