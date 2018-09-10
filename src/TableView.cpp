// =================================================================== //
// Copyright (C) 2014-2018 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "TableView.h"

#include <QtWidgets>
#include <QGraphicsPixmapItem>

#include <libbsdf/Brdf/Brdf.h>
#include <libbsdf/Brdf/Btdf.h>
#include <libbsdf/Brdf/SampleSet2D.h>
#include <libbsdf/Brdf/SphericalCoordinatesBrdf.h>

#include <libbsdf/Common/Utility.h>
#include <libbsdf/Common/SpectrumUtility.h>

#include "GraphicsAngleItem.h"
#include "GraphicsSampleItem.h"
#include "SceneUtil.h"

TableView::TableView(QWidget* parent) : QGraphicsView(parent),
                                        data_(0),
                                        backSideShown_(true)
{
    actionFitView_ = new QAction(this);
    actionFitView_->setText(QApplication::translate("TableView", "Fit in view", 0));
    connect(actionFitView_, SIGNAL(triggered()), this, SLOT(fitView()));

    actionShowBackSide_ = new QAction(this);
    actionShowBackSide_->setText(QApplication::translate("TableView", "Show/Hide back side samples", 0));
    connect(actionShowBackSide_, SIGNAL(triggered()), this, SLOT(changeBackSideVisibility()));

    graphicsScene_ = new QGraphicsScene;
    setScene(graphicsScene_);
}

void TableView::createTable(int wavelengthIndex, float gamma, bool photometric)
{
    if (!data_) return;

    wavelengthIndex_ = wavelengthIndex;
    gamma_ = gamma;
    photometric_ = photometric;

    const lb::Brdf* brdf = 0;
    if (data_->getBrdf()) {
        brdf = data_->getBrdf();
    }
    else if (data_->getBtdf()) {
        brdf = data_->getBtdf()->getBrdf();
    }

    if (!brdf ||
        (brdf && dynamic_cast<const lb::SphericalCoordinatesBrdf*>(brdf))) {
        actionShowBackSide_->setDisabled(true);
    }
    else {
        actionShowBackSide_->setEnabled(true);
    }

    graphicsScene_->clear();

    const qreal sceneSize = 10000000.0;
    scene()->setSceneRect(-sceneSize / 2.0, -sceneSize / 2.0, sceneSize, sceneSize);

    if (data_->getBrdf() || data_->getBtdf()) {
        createBrdfTable(wavelengthIndex);
    }
    else if (data_->getSpecularReflectances() || data_->getSpecularTransmittances()) {
        createReflectanceTable(wavelengthIndex);
    }
}

void TableView::createBrdfTable(int wavelengthIndex)
{
    const lb::SampleSet* ss = data_->getSampleSet();
    if (!ss) return;

    int numSamples = ss->getNumAngles0() * ss->getNumAngles1() * ss->getNumAngles2() * ss->getNumAngles3();
    if (numSamples < 100000) {
        createBrdfDataItems(wavelengthIndex);
    }
    else {
        createBrdfDataPixmapItem(wavelengthIndex);
    }

    createBrdfAngleItems(ss);
}

void TableView::createBrdfDataItems(int wavelengthIndex)
{
    const lb::Brdf* brdf = 0;
    const lb::SampleSet2D* ss2 = 0;
    if (data_->getBrdf()) {
        brdf = data_->getBrdf();
    }
    else if (data_->getBtdf()) {
        brdf = data_->getBtdf()->getBrdf();
    }
    else {
        return;
    }

    const lb::SampleSet* ss = brdf->getSampleSet();

    int num0 = ss->getNumAngles0();
    int num1 = ss->getNumAngles1();
    int num2 = ss->getNumAngles2();
    int num3 = ss->getNumAngles3();

    QList<QGraphicsItem*> whiteItems, blackItems;

    // data table
    for (int i0 = 0; i0 < num0; ++i0) {
    for (int i1 = 0; i1 < num1; ++i1) {
    for (int i2 = 0; i2 < num2; ++i2) {
    for (int i3 = 0; i3 < num3; ++i3) {
        if (!backSideShown_) {
            lb::Vec3 inDir, outDir;
            brdf->getInOutDirection(i0, i1, i2, i3, &inDir, &outDir);
            if (lb::isDownwardDir(inDir) || lb::isDownwardDir(outDir)) {
                continue;
            }
        }

        float sampleValue = getSampleValue(ss->getSpectrum(i0, i1, i2, i3),
                                           ss->getColorModel(),
                                           ss->getWavelengths(),
                                           wavelengthIndex);

        float maxValue = data_->getMaxValuesPerWavelength()[wavelengthIndex];
        int color;
        if (maxValue == 0.0f) {
            color = 0;
        }
        else {
            color = static_cast<int>(std::pow(sampleValue / maxValue, 1.0f / gamma_) * 255.0f);
            color = std::min(color, 255);
        }

        GraphicsSampleItem* item = new GraphicsSampleItem(QColor(color, color, color), sampleValue);
        item->setPos(i2 + num2 * i0, i3 + num3 * i1);

        // Classify items for the display order.
        if (item->isWhitish()) {
            whiteItems.push_back(item);
        }
        else {
            blackItems.push_back(item);
        }
    }}}}

    for (auto it = whiteItems.begin(); it != whiteItems.end(); ++it) {
        graphicsScene_->addItem(*it);
    }

    for (auto it = blackItems.begin(); it != blackItems.end(); ++it) {
        graphicsScene_->addItem(*it);
    }
}

void TableView::createBrdfDataPixmapItem(int wavelengthIndex)
{
    const lb::Brdf* brdf = 0;
    const lb::SampleSet2D* ss2 = 0;
    if (data_->getBrdf()) {
        brdf = data_->getBrdf();
    }
    else if (data_->getBtdf()) {
        brdf = data_->getBtdf()->getBrdf();
    }
    else {
        return;
    }

    const lb::SampleSet* ss = brdf->getSampleSet();

    int num0 = ss->getNumAngles0();
    int num1 = ss->getNumAngles1();
    int num2 = ss->getNumAngles2();
    int num3 = ss->getNumAngles3();

    QImage* image = new QImage(num0 * num2, num1 * num3, QImage::Format_RGB888);

    // data table
    for (int i0 = 0; i0 < num0; ++i0) {
    for (int i1 = 0; i1 < num1; ++i1) {
    for (int i2 = 0; i2 < num2; ++i2) {
        lb::Vec3 inDir, outDir;
        float sampleValue, maxValue;
        int color;
        #pragma omp parallel for private(sampleValue, maxValue, color, inDir, outDir)
        for (int i3 = 0; i3 < num3; ++i3) {
            if (!backSideShown_) {
                brdf->getInOutDirection(i0, i1, i2, i3, &inDir, &outDir);
                if (lb::isDownwardDir(inDir) || lb::isDownwardDir(outDir)) {
                    image->setPixel(i2 + num2 * i0, i3 + num3 * i1, backgroundBrush().color().rgb());
                    continue;
                }
            }

            sampleValue = getSampleValue(ss->getSpectrum(i0, i1, i2, i3),
                                         ss->getColorModel(),
                                         ss->getWavelengths(),
                                         wavelengthIndex);
            
            maxValue = data_->getMaxValuesPerWavelength()[wavelengthIndex];
            if (maxValue == 0.0f) {
                color = 0;
            }
            else {
                color = static_cast<int>(std::pow(sampleValue / maxValue, 1.0f / gamma_) * 255.0f);
                color = std::min(color, 255);
            }

            image->setPixel(i2 + num2 * i0, i3 + num3 * i1, QColor(color, color, color).rgb());
        }
    }}}

    QGraphicsPixmapItem* pixmapItem = new QGraphicsPixmapItem;
    pixmapItem->setPixmap(QPixmap::fromImage(*image));

    delete image;

    graphicsScene_->addItem(pixmapItem);
}

void TableView::createBrdfAngleItems(const lb::SampleSet* ss)
{
    int num0 = ss->getNumAngles0();
    int num1 = ss->getNumAngles1();
    int num2 = ss->getNumAngles2();
    int num3 = ss->getNumAngles3();

    const qreal itemSize = 1.0;

    // x-asis angles
    for (int i0 = 0; i0 < num0; ++i0) {
    for (int i2 = 0; i2 < num2; ++i2) {
        float angle0 = lb::toDegree(ss->getAngle0(i0));
        QColor color0 = (i0 % 2) ? QColor(Qt::red).lighter(170) : QColor(Qt::red).lighter(190);
        addAngleItem(color0, angle0, i2 + num2 * i0, -itemSize - itemSize, 0.0);
        addAngleItem(color0, angle0, i2 + num2 * i0, num1 * num3 * itemSize + itemSize);

        float angle2 = lb::toDegree(ss->getAngle2(i2));
        QColor color2 = (i2 % 2) ? QColor(Qt::green).lighter(170) : QColor(Qt::green).lighter(190);
        addAngleItem(color2, angle2, i2 + num2 * i0, -itemSize, 0.0);
        addAngleItem(color2, angle2, i2 + num2 * i0, num1 * num3 * itemSize);
    }}

    // y-asis angles
    for (int i1 = 0; i1 < num1; ++i1) {
    for (int i3 = 0; i3 < num3; ++i3) {
        if (num1 > 1) {
            float angle1 = lb::toDegree(ss->getAngle1(i1));
            QColor color1 = (i1 % 2) ? QColor(Qt::yellow).lighter(170) : QColor(Qt::yellow).lighter(190);
            addAngleItem(color1, angle1, -itemSize - itemSize, i3 + num3 * i1, 0.0);
            addAngleItem(color1, angle1, num0 * num2 * itemSize + itemSize, i3 + num3 * i1);
        }

        float angle3 = lb::toDegree(ss->getAngle3(i3));
        QColor color3 = (i3 % 2) ? QColor(Qt::blue).lighter(170) : QColor(Qt::blue).lighter(190);
        addAngleItem(color3, angle3, -itemSize, i3 + num3 * i1, 0.0);
        addAngleItem(color3, angle3, num0 * num2 * itemSize, i3 + num3 * i1);
    }}
}

void TableView::createReflectanceTable(int wavelengthIndex)
{
    const lb::SampleSet2D* ss2;
    if (data_->getSpecularReflectances()) {
        ss2 = data_->getSpecularReflectances();
    }
    else if  (data_->getSpecularTransmittances()) {
        ss2 = data_->getSpecularTransmittances();
    }
    else {
        return;
    }
    
    int num0 = ss2->getNumTheta();
    int num1 = ss2->getNumPhi();

    const qreal itemSize = 1.0;

    QList<QGraphicsItem*> whiteItems, blackItems;

    // data table
    for (int i0 = 0; i0 < num0; ++i0) {
    for (int i1 = 0; i1 < num1; ++i1) {
        float sampleValue = getSampleValue(ss2->getSpectrum(i0, i1),
                                           ss2->getColorModel(),
                                           ss2->getWavelengths(),
                                           wavelengthIndex);

        float value = std::pow(sampleValue, 1.0f / gamma_) * 255.0f;

        GraphicsSampleItem* item = new GraphicsSampleItem(QColor(value, value, value), sampleValue);
        item->setPos(i0, i1);

        if (item->isWhitish()) {
            whiteItems.push_back(item);
        }
        else {
            blackItems.push_back(item);
        }
    }}

    for (auto it = whiteItems.begin(); it != whiteItems.end(); ++it) {
        graphicsScene_->addItem(*it);
    }

    for (auto it = blackItems.begin(); it != blackItems.end(); ++it) {
        graphicsScene_->addItem(*it);
    }

    // x-asis angles
    for (int i0 = 0; i0 < num0; ++i0) {
        float angle0 = lb::toDegree(ss2->getTheta(i0));
        QColor color0 = (i0 % 2) ? QColor(Qt::red).lighter(170) : QColor(Qt::red).lighter(190);
        addAngleItem(color0, angle0, i0, -itemSize, 0.0);

        if (num1 > 1) {
            addAngleItem(color0, angle0, i0, num1 * itemSize);
        }
    }

    // y-asis angles
    if (num1 > 1) {
        for (int i1 = 0; i1 < num1; ++i1) {
            float angle1 = lb::toDegree(ss2->getPhi(i1));
            QColor color1 = (i1 % 2) ? QColor(Qt::yellow).lighter(170) : QColor(Qt::yellow).lighter(190);
            addAngleItem(color1, angle1, -itemSize, i1, 0.0);
            addAngleItem(color1, angle1, num0 * itemSize, i1);
        }
    }
}

float TableView::getSampleValue(const lb::Spectrum& sp,
                                lb::ColorModel      colorModel,
                                const lb::Arrayf&   wavelengths,
                                int                 wavelengthIndex)
{
    float value;
    if (photometric_) {
        value = scene_util::spectrumToY(sp, colorModel, wavelengths);
    }
    else {
        value = sp[wavelengthIndex];
    }

    return std::max(value, 0.0f);
}

void TableView::addAngleItem(const QColor& color, float angle,
                             qreal posX, qreal posY,
                             qreal lodThreshold, qreal textLodThreshold)
{
    GraphicsAngleItem* item = new GraphicsAngleItem(color, angle);
    item->setPos(posX, posY);
    item->setLodThreshold(lodThreshold);
    item->setTextLodThreshold(textLodThreshold);
    graphicsScene_->addItem(item);
}

void TableView::paintEvent(QPaintEvent* event)
{
    QGraphicsView::paintEvent(event);

    const lb::Brdf* brdf = 0;
    const lb::SampleSet2D* ss2 = 0;
    if (data_->getBrdf()) {
        brdf = data_->getBrdf();
    }
    else if (data_->getBtdf()) {
        brdf = data_->getBtdf()->getBrdf();
    }
    else if (data_->getSpecularReflectances()) {
        ss2 = data_->getSpecularReflectances();
    }
    else if (data_->getSpecularTransmittances()) {
        ss2 = data_->getSpecularTransmittances();
    }
    else {
        return;
    }

    QPainter painter(viewport());

    painter.save();

    QFont font("Times");
    font.setStyleStrategy(QFont::ForceOutline);
    painter.setFont(font);

    QRect angleNameRect(0, 0, 150, 18);

    if (brdf) {
        const lb::SampleSet* ss = brdf->getSampleSet();

        int numAngle1 = ss->getNumAngles1();

        if (numAngle1 == 1) {
            painter.translate(angleNameRect.height() + 1, 1);
        }
        else {
            painter.translate(angleNameRect.height() * 2 + 1, 1);
        }

        painter.fillRect(angleNameRect, QColor(Qt::red).lighter(180));
        painter.drawRect(angleNameRect);
        painter.drawText(angleNameRect, Qt::AlignCenter, brdf->getAngle0Name().c_str());

        painter.translate(0, angleNameRect.height());
        painter.fillRect(angleNameRect, QColor(Qt::green).lighter(180));
        painter.drawRect(angleNameRect);
        painter.drawText(angleNameRect, Qt::AlignCenter, brdf->getAngle2Name().c_str());

        painter.resetTransform();
        painter.rotate(-90.0);

        if (numAngle1 == 1) {
            painter.translate(-angleNameRect.width() - angleNameRect.height() * 2 - 1, -angleNameRect.height() + 1);
        }
        else {
            painter.translate(-angleNameRect.width() - angleNameRect.height() * 2 - 1, 1);
            painter.fillRect(angleNameRect, QColor(Qt::yellow).lighter(180));
            painter.drawRect(angleNameRect);
            painter.drawText(angleNameRect, Qt::AlignCenter, brdf->getAngle1Name().c_str());
        }

        painter.translate(0, angleNameRect.height());
        painter.fillRect(angleNameRect, QColor(Qt::blue).lighter(180));
        painter.drawRect(angleNameRect);
        painter.drawText(angleNameRect, Qt::AlignCenter, brdf->getAngle3Name().c_str());
    }
    else if (ss2) {
        int numInPhi = ss2->getNumPhi();

        if (numInPhi == 1) {
            painter.translate(1, 1);
        }
        else {
            painter.translate(angleNameRect.height() + 1, 1);
        }
        
        painter.fillRect(angleNameRect, QColor(Qt::red).lighter(180));
        painter.drawRect(angleNameRect);
        painter.drawText(angleNameRect, Qt::AlignCenter, "Incoming polar angle");

        if (numInPhi > 1) {
            painter.resetTransform();
            painter.rotate(-90.0);

            painter.translate(-angleNameRect.width() - angleNameRect.height() - 1, 1);
            painter.fillRect(angleNameRect, QColor(Qt::yellow).lighter(180));
            painter.drawRect(angleNameRect);
            painter.drawText(angleNameRect, Qt::AlignCenter, "Incoming azimuthal angle");
        }
    }

    painter.restore();
}

void TableView::wheelEvent(QWheelEvent* event)
{
    if (event->delta() > 0) {
        scale(0.9, 0.9);
    }
    else {
        scale(1.0 / 0.9, 1.0 / 0.9);
    }
    event->accept();
}

void TableView::contextMenuEvent(QContextMenuEvent* event)
{
    QMenu menu(this);
    menu.addAction(actionFitView_);
    menu.addAction(actionShowBackSide_);
    menu.exec(event->globalPos());
}
