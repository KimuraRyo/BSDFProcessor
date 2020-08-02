// =================================================================== //
// Copyright (C) 2014-2020 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "GraphicsSampleItem.h"

#include <QtWidgets>

GraphicsSampleItem::GraphicsSampleItem(const QColor&    color,
                                       float            value)
                                       : color_(color),
                                         value_(value),
                                         width_(1),
                                         height_(1)
{
    //setFlags(ItemIsSelectable);
    //setAcceptsHoverEvents(true);

    if (color.valueF() < 0.5) {
        textColor_ = QColor(Qt::white).darker(120);
        gridColor_ = QColor(Qt::gray).darker(120);
        whitish_ = false;
    }
    else {
        textColor_ = QColor(Qt::darkGray).darker(260);
        gridColor_ = QColor(Qt::gray).darker(130);
        whitish_ = true;
    }
}

QRectF GraphicsSampleItem::boundingRect() const
{
    return QRectF(0, 0, width_, height_);
}

QPainterPath GraphicsSampleItem::shape() const
{
    QPainterPath path;
    path.addRect(boundingRect());
    return path;
}

void GraphicsSampleItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    Q_UNUSED(widget);

    painter->save();

    painter->fillRect(boundingRect(), color_);

    qreal lod = option->levelOfDetailFromTransform(painter->worldTransform());

    if (lod > 40.0) {
        painter->setPen(QPen(gridColor_, 0));
        painter->drawRect(boundingRect());

        painter->setPen(QPen(textColor_));

        int precision = std::min(static_cast<int>(lod / 20.0), 6);
        QString text;
        if (value_ < 1.0f || value_ >= 10000.0f) {
            text = QString::number(value_, 'g', precision);
        }
        else {
            text = QString::number(value_, 'f', precision);
        }

        QFont font("Helvetica");
        font.setStyleStrategy(QFont::ForceOutline);
        painter->setFont(font);

        qreal scaleCoeff = 1.0 / lod;
        painter->scale(scaleCoeff, scaleCoeff);

        QRectF textRect(0, 0, width_ / scaleCoeff, height_ / scaleCoeff);
        painter->drawText(textRect, Qt::AlignCenter, text);
    }

    painter->restore();
}
