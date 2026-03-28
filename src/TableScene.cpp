// =================================================================== //
// Copyright (C) 2019-2026 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "TableScene.h"

#include <libbsdf/Common/SpectrumUtility.h>

#include "GraphicsAngleItem.h"
#include "GraphicsSampleItem.h"

constexpr int ITEM_SIZE = 1.0;
constexpr qreal ALWAYS_VISIBLE_LOD = 0.0;

TableScene::TableScene(QObject* parent)
    : QGraphicsScene(parent),
      brdfImage_(nullptr),
      data_(nullptr),
      backgroundColor_(QColor(107, 107, 133)),
      wavelengthIndex_(0),
      gamma_(2.2f),
      photometric_(true),
      backSideShown_(true)
{
}

float TableScene::getSampleValue(const lb::Spectrum& sp,
                                 lb::ColorModel      colorModel,
                                 const lb::Arrayf&   wavelengths)
{
    if (photometric_) {
        return lb::SpectrumUtility::spectrumToY(sp, colorModel, wavelengths);
    }
    else {
        return sp[wavelengthIndex_];
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

    if (!brdfImage_) {
        return;
    }

    // Convert the drawing area (floating-point) displayed on the screen to an integer rectangle that fits within the image area.
    QRect drawRect = rect.toAlignedRect().intersected(brdfImage_->rect());
    if (drawRect.isEmpty()) {
        return;
    }

    painter->setRenderHint(QPainter::SmoothPixmapTransform, false);
    painter->drawImage(drawRect.topLeft(), *brdfImage_, drawRect);

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
                QRgb pixel = brdfImage_->pixel(x, y);

                const lb::SampleSet* ss = data_->getBrdfData()->getSampleSet();
                int                  i0, i1, i2, i3;
                if (!getIndex(QPointF(x, y), &i0, &i1, &i2, &i3)) {
                    continue;
                }
                float sampleValue = getSampleValue(ss->getSpectrum(i0, i1, i2, i3),
                                                   ss->getColorModel(), ss->getWavelengths());

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

void TableScene::initTable(int wavelengthIndex, float gamma, bool photometric)
{
    if (!data_)
        return;

    wavelengthIndex_ = wavelengthIndex;
    gamma_ = gamma;
    photometric_ = photometric;

    clear();

    if (data_->getBrdf() || data_->getBtdf()) {
        createBrdfTable();
    }
    else if (data_->getSpecularReflectances() || data_->getSpecularTransmittances()) {
        // Free the image data so that the drawBackground function is not executed.
        brdfImage_.reset();

        createReflectanceTable();
    }

    constexpr qreal sceneSize = 10000000.0;
    setSceneRect(-sceneSize / 2.0, -sceneSize / 2.0, sceneSize, sceneSize);
}

void TableScene::updateTable()
{
    if (!data_)
        return;

    clear();

    if (data_->getBrdf() || data_->getBtdf()) {
        createBrdfTable();
    }
    else if (data_->getSpecularReflectances() || data_->getSpecularTransmittances()) {
        createReflectanceTable();
    }

    constexpr qreal sceneSize = 10000000.0;
    setSceneRect(-sceneSize / 2.0, -sceneSize / 2.0, sceneSize, sceneSize);
}

bool TableScene::getInOutDir(const QPointF& pos, lb::Vec3* inDir, lb::Vec3* outDir)
{
    int i0, i1, i2, i3;
    if (!getIndex(pos, &i0, &i1, &i2, &i3)) {
        *inDir = lb::Vec3::Zero();
        *outDir = lb::Vec3::Zero();
        return false;
    }

    if (const lb::Brdf* brdf = data_->getBrdfData()) {
        brdf->getInOutDirection(i0, i1, i2, i3, inDir, outDir);

        if (brdf->getSampleSet()->isIsotropic()) {
            float inTheta, outTheta, outPhi;
            lb::SphericalCoordinateSystem::fromXyz(*inDir, *outDir, &inTheta, &outTheta, &outPhi);
            lb::SphericalCoordinateSystem::toXyz(inTheta, 0.0f, outTheta, outPhi, inDir, outDir);
        }
    }
    else if (const lb::SampleSet2D* ss2 = data_->getSampleSet2D()) {
        float theta = ss2->getTheta(i0);
        float phi = ss2->getTheta(i1);

        *inDir = lb::SphericalCoordinateSystem::toXyz(theta, phi);
        *outDir = lb::reflect(*inDir, lb::Vec3(0, 0, 1));
    }
    else {
        return false;
    }

    return true;
}

void TableScene::createBrdfTable()
{
    const lb::SampleSet* ss = data_->getSampleSet();
    if (!ss) return;

    createBrdfImage(); 
    createBrdfAngleItems(*ss);
}

void TableScene::createBrdfImage()
{
    const lb::Brdf* brdf = data_->getBrdfData();
    if (!brdf) return;

    const lb::SampleSet* ss = brdf->getSampleSet();

    int num0 = ss->getNumAngles0();
    int num1 = ss->getNumAngles1();
    int num2 = ss->getNumAngles2();
    int num3 = ss->getNumAngles3();

    brdfImage_ = std::make_unique<QImage>(num0 * num2, num1 * num3, QImage::Format_RGB888);

    // data table
    for (int i0 = 0; i0 < num0; ++i0) {
    for (int i1 = 0; i1 < num1; ++i1) {
    for (int i2 = 0; i2 < num2; ++i2) {
        lb::Vec3 inDir, outDir;
        float sampleValue, maxValue, ratio;
        int color;
        #pragma omp parallel for private(sampleValue, maxValue, ratio, color, inDir, outDir)
        for (int i3 = 0; i3 < num3; ++i3) {
            if (!backSideShown_) {
                brdf->getInOutDirection(i0, i1, i2, i3, &inDir, &outDir);
                if (lb::isDownwardDir(inDir) || lb::isDownwardDir(outDir)) {
                    brdfImage_->setPixelColor(i2 + num2 * i0, i3 + num3 * i1, backgroundColor_);
                    continue;
                }
            }

            sampleValue = getSampleValue(ss->getSpectrum(i0, i1, i2, i3), ss->getColorModel(),
                                         ss->getWavelengths());
            if (sampleValue < lb::EPSILON_F) {
                color = 0;
            }
            else {
                maxValue = data_->getMaxValuesPerWavelength()[wavelengthIndex_];
                ratio = sampleValue / maxValue;
                color = static_cast<int>(std::pow(ratio, 1 / gamma_) * 255);
                color = std::min(color, 255);
            }

            brdfImage_->setPixelColor(i2 + num2 * i0, i3 + num3 * i1, QColor(color, color, color));
        }
    }}}
}

void TableScene::createBrdfAngleItems(const lb::SampleSet& ss)
{
    int num0 = ss.getNumAngles0();
    int num1 = ss.getNumAngles1();
    int num2 = ss.getNumAngles2();
    int num3 = ss.getNumAngles3();

    QPen linePen;
    linePen.setWidthF(0.05);
    linePen.setStyle(Qt::DashLine);
    linePen.setCapStyle(Qt::FlatCap);

    // x-axis angles
    qreal underPosY = num1 * num3 * ITEM_SIZE;
    linePen.setColor(QColor(Qt::red).lighter(140));
    for (int i0 = 0; i0 < num0; ++i0) {
        for (int i2 = 0; i2 < num2; ++i2) {
            qreal posX = i2 + num2 * i0;

            QColor color0 = (i0 % 2) ? QColor(Qt::red).lighter(170) : QColor(Qt::red).lighter(190);
            float  angle0 = lb::toDegree(ss.getAngle0(i0));
            addAngleItem(color0, angle0, posX, -ITEM_SIZE * 2, ALWAYS_VISIBLE_LOD); // upper side
            addAngleItem(color0, angle0, posX, underPosY + ITEM_SIZE);              // under side

            QColor color2 =
                (i2 % 2) ? QColor(Qt::green).lighter(170) : QColor(Qt::green).lighter(190);
            float angle2 = lb::toDegree(ss.getAngle2(i2));
            addAngleItem(color2, angle2, posX, -ITEM_SIZE, ALWAYS_VISIBLE_LOD); // upper side
            addAngleItem(color2, angle2, posX, underPosY);                      // under side

            // Add a boundary line.
            if (i0 >= 1 && i2 == 0) {
                addLine(posX, 0, posX, underPosY, linePen);
            }
        }}

    // y-axis angles
    qreal rightPosX = num0 * num2 * ITEM_SIZE;
    linePen.setColor(QColor(Qt::yellow).lighter(140));
    for (int i1 = 0; i1 < num1; ++i1) {
        for (int i3 = 0; i3 < num3; ++i3) {
            qreal posY = i3 + num3 * i1;

            if (num1 > 1) {
                QColor color1 =
                    (i1 % 2) ? QColor(Qt::yellow).lighter(170) : QColor(Qt::yellow).lighter(190);
                float angle1 = lb::toDegree(ss.getAngle1(i1));
                addAngleItem(color1, angle1, -ITEM_SIZE * 2, posY, ALWAYS_VISIBLE_LOD); // left side
                addAngleItem(color1, angle1, rightPosX + ITEM_SIZE, posY); // right side
            }

            QColor color3 =
                (i3 % 2) ? QColor(Qt::blue).lighter(170) : QColor(Qt::blue).lighter(190);
            float angle3 = lb::toDegree(ss.getAngle3(i3));
            addAngleItem(color3, angle3, -ITEM_SIZE, posY, ALWAYS_VISIBLE_LOD); // left side
            addAngleItem(color3, angle3, rightPosX, posY);                      // right side

            // Add a boundary line.
            if (i1 >= 1 && i3 == 0) {
                addLine(0, posY, rightPosX, posY, linePen);
            }
        }}
}

void TableScene::createReflectanceTable()
{
    const lb::SampleSet2D* ss2;
    if (data_->getSpecularReflectances()) {
        ss2 = data_->getSpecularReflectances().get();
    }
    else if (data_->getSpecularTransmittances()) {
        ss2 = data_->getSpecularTransmittances().get();
    }
    else {
        return;
    }

    createReflectanceDataItems(*ss2);
    createReflectanceAngleItems(*ss2);
}

void TableScene::createReflectanceDataItems(const lb::SampleSet2D& ss2)
{
    int num0 = ss2.getNumTheta();
    int num1 = ss2.getNumPhi();

    QList<QGraphicsItem*> whiteItems, blackItems;

    // data table
    for (int i0 = 0; i0 < num0; ++i0) {
        for (int i1 = 0; i1 < num1; ++i1) {
            float sampleValue =
                getSampleValue(ss2.getSpectrum(i0, i1), ss2.getColorModel(), ss2.getWavelengths());
            float value = std::pow(lb::clamp(sampleValue, 0.0f, 1.0f), 1 / gamma_) * 255;

            GraphicsSampleItem* item =
                new GraphicsSampleItem(QColor(value, value, value), sampleValue);
            item->setPos(i0, i1);

            if (item->isWhitish()) {
                whiteItems.push_back(item);
            }
            else {
                blackItems.push_back(item);
            }
        }
    }

    for (auto it = whiteItems.begin(); it != whiteItems.end(); ++it) { addItem(*it); }
    for (auto it = blackItems.begin(); it != blackItems.end(); ++it) { addItem(*it); }
}

void TableScene::createReflectanceAngleItems(const lb::SampleSet2D& ss2)
{
    int num0 = ss2.getNumTheta();
    int num1 = ss2.getNumPhi();

    const qreal itemSize = 1.0;

    // x-axis angles
    for (int i0 = 0; i0 < num0; ++i0) {
        QColor color0 = (i0 % 2) ? QColor(Qt::red).lighter(170) : QColor(Qt::red).lighter(190);
        float  angle0 = lb::toDegree(ss2.getTheta(i0));
        addAngleItem(color0, angle0, i0, -itemSize, ALWAYS_VISIBLE_LOD);

        if (num1 > 1) {
            addAngleItem(color0, angle0, i0, num1 * itemSize);
        }
    }

    // y-axis angles
    if (num1 > 1) {
        for (int i1 = 0; i1 < num1; ++i1) {
            QColor color1 =
                (i1 % 2) ? QColor(Qt::yellow).lighter(170) : QColor(Qt::yellow).lighter(190);
            float angle1 = lb::toDegree(ss2.getPhi(i1));
            addAngleItem(color1, angle1, -itemSize, i1, ALWAYS_VISIBLE_LOD); // left side
            addAngleItem(color1, angle1, num0 * itemSize, i1);               // right side
        }
    }
}

void TableScene::addAngleItem(const QColor& color,
                              float         angle,
                              qreal         posX,
                              qreal         posY,
                              qreal         lodThreshold,
                              qreal         textLodThreshold)
{
    GraphicsAngleItem* item = new GraphicsAngleItem(color, angle);
    item->setPos(posX, posY);
    item->setLodThreshold(lodThreshold);
    item->setTextLodThreshold(textLodThreshold);
    addItem(item);
}