// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "GraphicsAngleItem.h"

#include <QtWidgets>

GraphicsAngleItem::GraphicsAngleItem(const QColor&  color,
                                     float          value)
                                     : color_(color),
                                       value_(value),
                                       width_(1),
                                       height_(1),
                                       lodThreshold_(25.0),
                                       textLodThreshold_(25.0)
{
    //setFlags(ItemIsSelectable);
    //setAcceptsHoverEvents(true);
}

QRectF GraphicsAngleItem::boundingRect() const
{
    return QRectF(0, 0, width_, height_);
}

QPainterPath GraphicsAngleItem::shape() const
{
    QPainterPath path;
    path.addRect(boundingRect());
    return path;
}

void GraphicsAngleItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    Q_UNUSED(widget);

    qreal lod = option->levelOfDetailFromTransform(painter->worldTransform());

    if (lod > lodThreshold_) {
        painter->save();

        painter->fillRect(boundingRect(), color_);

        if (lod > textLodThreshold_) {
            painter->setPen(QPen(QColor(Qt::gray), 0));
            painter->drawRect(boundingRect());

            painter->setPen(QPen(QColor(Qt::black)));

            int precision = std::min(static_cast<int>(lod / 8.0), 5);
            QString text = QString::number(value_, 'g', precision);

            QFont font("Times");
            font.setStyleStrategy(QFont::ForceOutline);
            painter->setFont(font);

            qreal scaleCoeff = 1.0 / lod;
            painter->scale(scaleCoeff, scaleCoeff);

            QRectF textRect(0, 0, width_ / scaleCoeff, height_ / scaleCoeff);
            painter->drawText(textRect, Qt::AlignCenter, text);
        }

        painter->restore();
    }
}
