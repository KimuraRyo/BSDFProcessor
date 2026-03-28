// =================================================================== //
// Copyright (C) 2019-2026 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "TableScene.h"

#include <libbsdf/Common/SpectrumUtility.h>

constexpr int ITEM_SIZE = 1.0;

TableScene::TableScene(QObject* parent)
    : QGraphicsScene(parent),
      data_(nullptr),
      backgroundColor_(QColor(107, 107, 133)),
      backSideShown_(true)
{
}

float TableScene::getSampleValue(const lb::Spectrum& sp,
                                 lb::ColorModel      colorModel,
                                 const lb::Arrayf&   wavelengths,
                                 int                 wavelengthIndex)
{
    if (photometric_) {
        return lb::SpectrumUtility::spectrumToY(sp, colorModel, wavelengths);
    }
    else {
        return sp[wavelengthIndex];
    }
}

bool TableScene::getIndex(const QPointF& pos, int* i0, int* i1, int* i2, int* i3)
{
    if (!data_) return false;

    if (const lb::SampleSet* ss = data_->getSampleSet()) {
        int num0 = ss->getNumAngles0();
        int num1 = ss->getNumAngles1();
        int num2 = ss->getNumAngles2();
        int num3 = ss->getNumAngles3();

        if (pos.x() < 0 || pos.x() > num0 * num2 * ITEM_SIZE ||
            pos.y() < 0 || pos.y() > num1 * num3 * ITEM_SIZE) {
            return false;
        }

        int x = static_cast<int>(pos.x());
        int y = static_cast<int>(pos.y());

        *i0 = x * ITEM_SIZE / num2;
        *i1 = y * ITEM_SIZE / num3;
        *i2 = x % num2;
        *i3 = y % num3;
    }
    else if (const lb::SampleSet2D* ss2 = data_->getSampleSet2D()) {
        int numTh = ss2->getNumTheta();
        int numPh = ss2->getNumPhi();

        if (pos.x() < 0 || pos.x() > numTh * ITEM_SIZE ||
            pos.y() < 0 || pos.y() > numPh * ITEM_SIZE) {
            return false;
        }

        int x = static_cast<int>(pos.x());
        int y = static_cast<int>(pos.y());

        *i0 = x * ITEM_SIZE;
        *i1 = y * ITEM_SIZE;
        *i2 = 0;
        *i3 = 0;
    }
    else {
        return false;
    }

    return true;
}

void TableScene::mouseMoveEvent(QGraphicsSceneMouseEvent* event)
{
    QGraphicsScene::mouseMoveEvent(event);

    emit mouseMoved(event->scenePos());
}

void TableScene::mouseReleaseEvent(QGraphicsSceneMouseEvent* event)
{
    QGraphicsScene::mouseReleaseEvent(event);

    if (event->button() == Qt::LeftButton &&
        event->screenPos() == event->lastScreenPos()) {
        emit mouseClicked(event->scenePos());
    }
}

void TableScene::drawBackground(QPainter* painter, const QRectF& rect)
{
    painter->fillRect(rect, backgroundColor_); 

    if (!data_ || image_.isNull()) {
        return;
    }

    // Convert the drawing area (floating-point) displayed on the screen to an integer rectangle that fits within the image area.
    QRect drawRect = rect.toAlignedRect().intersected(image_.rect());
    if (drawRect.isEmpty()) {
        return;
    }

    painter->setRenderHint(QPainter::SmoothPixmapTransform, false);
    painter->drawImage(drawRect.topLeft(), image_, drawRect);

    const qreal lod = painter->transform().m11();

    // Draw the grid if the zoom level is above a certain threshold (where 1 pixel corresponds to 20 or more pixels on the screen).
    if (lod > 20.0) {
        QVector<QLineF> lines;
        for (int x = drawRect.left(); x <= drawRect.right() + 1; ++x) {
            lines.append(QLineF(x, drawRect.top(), x, drawRect.bottom() + 1));
        }
        for (int y = drawRect.top(); y <= drawRect.bottom() + 1; ++y) {
            lines.append(QLineF(drawRect.left(), y, drawRect.right() + 1, y));
        }

        QPen backgroundGridPen(QColor(0, 0, 0, 150));
        backgroundGridPen.setCosmetic(true);
        backgroundGridPen.setWidth(3);
        painter->setPen(backgroundGridPen);
        painter->drawLines(lines);

        QPen gridPen(QColor(255, 255, 255, 150));
        gridPen.setCosmetic(true);
        gridPen.setWidth(1);
        painter->setPen(gridPen);
        painter->drawLines(lines);
    }

    // Draw the value when further zoomed in (when 1 pixel corresponds to 40px or more on the screen).
    if (lod > 40.0) {
        painter->save();

        QTransform transform = painter->transform();
        painter->resetTransform();

        QFont font = painter->font();
        font.setPixelSize(11);
        painter->setFont(font);
        QFontMetrics fm(font);

        for (int y = drawRect.top(); y <= drawRect.bottom(); ++y) {
            for (int x = drawRect.left(); x <= drawRect.right(); ++x) {
                QRgb pixel = image_.pixel(x, y);

                const lb::SampleSet* ss = data_->getBrdfData()->getSampleSet();
                int                  i0, i1, i2, i3;
                if (!getIndex(QPointF(x, y), &i0, &i1, &i2, &i3)) {
                    continue;
                }
                float sampleValue =
                    getSampleValue(ss->getSpectrum(i0, i1, i2, i3), ss->getColorModel(),
                                   ss->getWavelengths(), wavelengthIndex_);

                QString text;
                // Adjust the number of displayed digits from 6 to 1 so that it fits within the pixel width (lod).
                for (int p = 6; p >= 1; --p) {
                    text = QString::number(sampleValue, 'g', p);

                    // Determine whether the drawing width of the string fits within the pixel width (lod), including the left and right margins (8px).
                    if (fm.boundingRect(text).width() <= lod - 8) {
                        break;
                    }
                }

                if (qGray(pixel) > 127) {
                    painter->setPen(Qt::black);
                }
                else {
                    painter->setPen(Qt::white);
                }

                QPointF centerScenePos(x + 0.5, y + 0.5);
                QPointF centerViewPos = transform.map(centerScenePos);

                int textWidth = fm.boundingRect(text).width();
                int textHeight = fm.ascent() - fm.descent();

                // Calculate the offset to center text within a pixel.
                int drawX = centerViewPos.x() - textWidth / 2.0;
                int drawY = centerViewPos.y() + textHeight / 2.0;

                painter->drawText(drawX, drawY, text);
            }
        }
        painter->restore();
    }
}