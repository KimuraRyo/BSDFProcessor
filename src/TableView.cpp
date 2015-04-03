// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
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

#include "GraphicsAngleItem.h"
#include "GraphicsSampleItem.h"
#include "GraphScene.h"

TableView::TableView(QWidget* parent) : QGraphicsView(parent),
                                        graphScene_(0)
{
    actionFitView_ = new QAction(this);
    actionFitView_->setText(QApplication::translate("TableView", "Fit in view", 0));
    connect(actionFitView_, SIGNAL(triggered()), this, SLOT(fitView()));

    graphicsScene_ = new QGraphicsScene;
    setScene(graphicsScene_);
}

void TableView::createTable(int spectrumIndex, float gamma)
{
    if (!graphScene_) return;

    graphicsScene_->clear();

    const qreal sceneSize = 10000000.0;
    scene()->setSceneRect(-sceneSize / 2.0, -sceneSize / 2.0, sceneSize, sceneSize);

    if (graphScene_->getBrdf() || graphScene_->getBtdf()) {
        createBrdfTable(spectrumIndex, gamma);
    }
    else if (graphScene_->getSpecularReflectances() || graphScene_->getSpecularTransmittances()) {
        createReflectanceTable(spectrumIndex);
    }
}

void TableView::createBrdfTable(int spectrumIndex, float gamma)
{
    const lb::SampleSet* ss = graphScene_->getSampleSet();
    if (!ss) return;

    int numSamples = ss->getNumAngles0() * ss->getNumAngles1() * ss->getNumAngles2() * ss->getNumAngles3();
    if (numSamples < 100000) {
        createBrdfDataItems(ss, spectrumIndex, gamma);
    }
    else {
        createBrdfDataPixmapItem(ss, spectrumIndex, gamma);
    }

    createBrdfAngleItems(ss);
}

void TableView::createBrdfDataItems(const lb::SampleSet* ss, int spectrumIndex, float gamma)
{
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
        float sampleValue = ss->getSpectrum(i0, i1, i2, i3)[spectrumIndex];
        sampleValue = std::max(sampleValue, 0.0f);
        float maxValue = graphScene_->getMaxValuesPerWavelength()[spectrumIndex];
        
        int color;
        if (maxValue == 0.0f) {
            color = 0;
        }
        else {
            color = std::pow(sampleValue / maxValue, 1.0f / gamma) * 255.0f;
        }

        GraphicsSampleItem* item = new GraphicsSampleItem(QColor(color, color, color), sampleValue);
        item->setPos(i2 + num2 * i0, i3 + num3 * i1);

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

void TableView::createBrdfDataPixmapItem(const lb::SampleSet* ss, int spectrumIndex, float gamma)
{
    int num0 = ss->getNumAngles0();
    int num1 = ss->getNumAngles1();
    int num2 = ss->getNumAngles2();
    int num3 = ss->getNumAngles3();

    QImage* image = new QImage(num0 * num2, num1 * num3, QImage::Format_RGB888);

    // data table
    for (int i0 = 0; i0 < num0; ++i0) {
    for (int i1 = 0; i1 < num1; ++i1) {
    for (int i2 = 0; i2 < num2; ++i2) {
        float sampleValue, maxValue;
        int color;
        #pragma omp parallel for private(sampleValue, maxValue, color)
        for (int i3 = 0; i3 < num3; ++i3) {
            sampleValue = ss->getSpectrum(i0, i1, i2, i3)[spectrumIndex];
            sampleValue = std::max(sampleValue, 0.0f);
            maxValue = graphScene_->getMaxValuesPerWavelength()[spectrumIndex];

            if (maxValue == 0.0f) {
                color = 0;
            }
            else {
                color = static_cast<int>(std::pow(sampleValue / maxValue, 1.0f / gamma) * 255.0f);
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

void TableView::createReflectanceTable(int spectrumIndex, float gamma)
{
    const lb::SampleSet2D* ss2;
    if (graphScene_->getSpecularReflectances()) {
        ss2 = graphScene_->getSpecularReflectances();
    }
    else if  (graphScene_->getSpecularTransmittances()) {
        ss2 = graphScene_->getSpecularTransmittances();
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
        float sampleValue = ss2->getSpectrum(i0, i1)[spectrumIndex];
        sampleValue = std::max(sampleValue, 0.0f);
        float value = std::pow(sampleValue, 1.0f / gamma) * 255.0f;

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
    if (graphScene_->getBrdf()) {
        brdf = graphScene_->getBrdf();
    }
    else if (graphScene_->getBtdf()) {
        brdf = graphScene_->getBtdf()->getBrdf();
    }
    else if (graphScene_->getSpecularReflectances()) {
        ss2 = graphScene_->getSpecularReflectances();
    }
    else if (graphScene_->getSpecularTransmittances()) {
        ss2 = graphScene_->getSpecularTransmittances();
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

        int numAngle0 = ss->getNumAngles1();

        if (numAngle0 == 1) {
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

        if (numAngle0 == 1) {
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
    menu.exec(event->globalPos());
}
