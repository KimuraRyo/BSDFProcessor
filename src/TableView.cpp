// =================================================================== //
// Copyright (C) 2014-2020 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "TableView.h"

#include <QtWidgets>

#include <libbsdf/Brdf/SphericalCoordinatesBrdf.h>
#include <libbsdf/Common/SpectrumUtility.h>

#include "GraphicsAngleItem.h"
#include "GraphicsSampleItem.h"
#include "SceneUtil.h"
#include "Utility.h"

constexpr int ITEM_SIZE = 1.0;
constexpr qreal ALWAYS_VISIBLE_LOD = 0.0;

TableView::TableView(QWidget* parent) : QGraphicsView(parent),
                                        actionLogPlot_(nullptr),
                                        data_(nullptr),
                                        backSideShown_(true),
                                        fittingNeeded_(true)
{
    actionFitView_ = new QAction(this);
    actionFitView_->setText("Fit in view");
    connect(actionFitView_, SIGNAL(triggered()), this, SLOT(fitView()));

    actionShowBackSide_ = new QAction(this);
    actionShowBackSide_->setText("Back side samples");
    actionShowBackSide_->setCheckable(true);
    connect(actionShowBackSide_, SIGNAL(triggered()), this, SLOT(changeBackSideVisibility()));

    tableScene_ = new TableScene;
    setScene(tableScene_);

    connect(tableScene_, SIGNAL(mouseMoved(QPointF)),   this, SLOT(showToolTip(QPointF)));
    connect(tableScene_, SIGNAL(mouseClicked(QPointF)), this, SLOT(updateInOutDir(QPointF)));
}

void TableView::createTable(int wavelengthIndex, float gamma, bool photometric)
{
    if (!data_) return;

    wavelengthIndex_ = wavelengthIndex;
    gamma_ = gamma;
    photometric_ = photometric;

    const lb::Brdf* brdf = data_->getBrdfData();
    if (!brdf ||
        (brdf && dynamic_cast<const lb::SphericalCoordinatesBrdf*>(brdf))) {
        actionShowBackSide_->setDisabled(true);
    }
    else {
        actionShowBackSide_->setEnabled(true);
    }

    tableScene_->clear();

    const qreal sceneSize = 10000000.0;
    scene()->setSceneRect(-sceneSize / 2.0, -sceneSize / 2.0, sceneSize, sceneSize);

    if (data_->getBrdf() || data_->getBtdf()) {
        createBrdfTable(wavelengthIndex);
    }
    else if (data_->getSpecularReflectances() || data_->getSpecularTransmittances()) {
        createReflectanceTable(wavelengthIndex);
    }
}

void TableView::fitView(qreal scaleFactor)
{
    fitInView(scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
    scale(scaleFactor, scaleFactor);
}

void TableView::changeBackSideVisibility()
{
    backSideShown_ = !backSideShown_;
    actionShowBackSide_->setChecked(backSideShown_);
    createTable(wavelengthIndex_, gamma_, photometric_);
}

void TableView::showToolTip(const QPointF& pos)
{
    int i0, i1, i2, i3;
    if (!getIndex(pos, &i0, &i1, &i2, &i3)) {
        setToolTip("");
        return;
    }

    float val;

    if (const lb::SampleSet* ss = data_->getSampleSet()) {
        val = getSampleValue(ss->getSpectrum(i0, i1, i2, i3),
                             ss->getColorModel(),
                             ss->getWavelengths(),
                             wavelengthIndex_);
    }
    else if (const lb::SampleSet2D* ss2 = data_->getSampleSet2D()) {
        val = getSampleValue(ss2->getSpectrum(i0, i1),
                             ss2->getColorModel(),
                             ss2->getWavelengths(),
                             wavelengthIndex_);
    }
    else {
        return;
    }

    setToolTip(QString::number(val));
}

void TableView::updateInOutDir(const QPointF& pos)
{
    lb::Vec3 inDir, outDir;
    if (getInOutDir(pos, &inDir, &outDir)) {
        if (data_->getBtdf() || data_->getSpecularTransmittances()) {
            outDir[2] = -outDir[2];
        }
        outDir.normalize();

        emit inOutDirPicked(inDir, outDir);
    }
    else {
        emit clearPickedValue();
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

    createBrdfAngleItems(*ss);
}

void TableView::createBrdfDataItems(int wavelengthIndex)
{
    const lb::Brdf* brdf = data_->getBrdfData();
    if (!brdf) return;

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
        int color;
        if (sampleValue < lb::EPSILON_F) {
            color = 0;
        }
        else {
            const lb::Spectrum maxSp = data_->getMaxValuesPerWavelength();
            float maxValue = photometric_ ? lb::SpectrumUtility::spectrumToY(maxSp, ss->getColorModel(), ss->getWavelengths())
                                          : maxSp[wavelengthIndex];
            float ratio = sampleValue / maxValue;
            color = static_cast<int>(std::pow(ratio, 1 / gamma_) * 255);
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
        tableScene_->addItem(*it);
    }

    for (auto it = blackItems.begin(); it != blackItems.end(); ++it) {
        tableScene_->addItem(*it);
    }
}

void TableView::createBrdfDataPixmapItem(int wavelengthIndex)
{
    const lb::Brdf* brdf = data_->getBrdfData();
    if (!brdf) return;

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
        float sampleValue, maxValue, ratio;
        int color;
        #pragma omp parallel for private(sampleValue, maxValue, ratio, color, inDir, outDir)
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
            if (sampleValue < lb::EPSILON_F) {
                color = 0;
            }
            else {
                maxValue = data_->getMaxValuesPerWavelength()[wavelengthIndex];
                ratio = sampleValue / maxValue;
                color = static_cast<int>(std::pow(ratio, 1 / gamma_) * 255);
                color = std::min(color, 255);
            }

            image->setPixel(i2 + num2 * i0, i3 + num3 * i1, QColor(color, color, color).rgb());
        }
    }}}

    QGraphicsPixmapItem* pixmapItem = new QGraphicsPixmapItem;
    pixmapItem->setPixmap(QPixmap::fromImage(*image));

    delete image;

    tableScene_->addItem(pixmapItem);
}

void TableView::createBrdfAngleItems(const lb::SampleSet& ss)
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
        float angle0 = lb::toDegree(ss.getAngle0(i0));
        addAngleItem(color0, angle0, posX, -ITEM_SIZE * 2, ALWAYS_VISIBLE_LOD);  // upper side
        addAngleItem(color0, angle0, posX, underPosY + ITEM_SIZE);               // under side

        QColor color2 = (i2 % 2) ? QColor(Qt::green).lighter(170) : QColor(Qt::green).lighter(190);
        float angle2 = lb::toDegree(ss.getAngle2(i2));
        addAngleItem(color2, angle2, posX, -ITEM_SIZE, ALWAYS_VISIBLE_LOD);  // upper side
        addAngleItem(color2, angle2, posX, underPosY);                       // under side

        // Add a boundary line.
        if (i0 >= 1 && i2 == 0) {
            tableScene_->addLine(posX, 0, posX, underPosY, linePen);
        }
    }}

    // y-axis angles
    qreal rightPosX = num0 * num2 * ITEM_SIZE;
    linePen.setColor(QColor(Qt::yellow).lighter(140));
    for (int i1 = 0; i1 < num1; ++i1) {
    for (int i3 = 0; i3 < num3; ++i3) {
        qreal posY = i3 + num3 * i1;

        if (num1 > 1) {
            QColor color1 = (i1 % 2) ? QColor(Qt::yellow).lighter(170) : QColor(Qt::yellow).lighter(190);
            float angle1 = lb::toDegree(ss.getAngle1(i1));
            addAngleItem(color1, angle1, -ITEM_SIZE * 2,        posY, ALWAYS_VISIBLE_LOD);  // left side
            addAngleItem(color1, angle1, rightPosX + ITEM_SIZE, posY);                      // right side
        }

        QColor color3 = (i3 % 2) ? QColor(Qt::blue).lighter(170) : QColor(Qt::blue).lighter(190);
        float angle3 = lb::toDegree(ss.getAngle3(i3));
        addAngleItem(color3, angle3, -ITEM_SIZE, posY, ALWAYS_VISIBLE_LOD);  // left side
        addAngleItem(color3, angle3, rightPosX, posY);                      // right side

        // Add a boundary line.
        if (i1 >= 1 && i3 == 0) {
            tableScene_->addLine(0, posY, rightPosX, posY, linePen);
        }
    }}
}

void TableView::createReflectanceTable(int wavelengthIndex)
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

    createReflectanceDataItems(*ss2, wavelengthIndex);
    createReflectanceAngleItems(*ss2);
}

void TableView::createReflectanceDataItems(const lb::SampleSet2D& ss2, int wavelengthIndex)
{
    int num0 = ss2.getNumTheta();
    int num1 = ss2.getNumPhi();

    const qreal itemSize = 1.0;

    QList<QGraphicsItem*> whiteItems, blackItems;

    // data table
    for (int i0 = 0; i0 < num0; ++i0) {
    for (int i1 = 0; i1 < num1; ++i1) {
        float sampleValue = getSampleValue(ss2.getSpectrum(i0, i1),
                                           ss2.getColorModel(),
                                           ss2.getWavelengths(),
                                           wavelengthIndex);
        float value = std::pow(lb::clamp(sampleValue, 0.0f, 1.0f), 1 / gamma_) * 255;

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
        tableScene_->addItem(*it);
    }

    for (auto it = blackItems.begin(); it != blackItems.end(); ++it) {
        tableScene_->addItem(*it);
    }
}

void TableView::createReflectanceAngleItems(const lb::SampleSet2D& ss2)
{
    int num0 = ss2.getNumTheta();
    int num1 = ss2.getNumPhi();

    const qreal itemSize = 1.0;

    // x-axis angles
    for (int i0 = 0; i0 < num0; ++i0) {
        QColor color0 = (i0 % 2) ? QColor(Qt::red).lighter(170) : QColor(Qt::red).lighter(190);
        float angle0 = lb::toDegree(ss2.getTheta(i0));
        addAngleItem(color0, angle0, i0, -itemSize, ALWAYS_VISIBLE_LOD);

        if (num1 > 1) {
            addAngleItem(color0, angle0, i0, num1 * itemSize);
        }
    }

    // y-axis angles
    if (num1 > 1) {
        for (int i1 = 0; i1 < num1; ++i1) {
            QColor color1 = (i1 % 2) ? QColor(Qt::yellow).lighter(170) : QColor(Qt::yellow).lighter(190);
            float angle1 = lb::toDegree(ss2.getPhi(i1));
            addAngleItem(color1, angle1, -itemSize,       i1, ALWAYS_VISIBLE_LOD);  // left side
            addAngleItem(color1, angle1, num0 * itemSize, i1);                      // right side
        }
    }
}

float TableView::getSampleValue(const lb::Spectrum& sp,
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

bool TableView::getIndex(const QPointF& pos, int* i0, int* i1, int* i2, int* i3)
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

bool TableView::getInOutDir(const QPointF& pos, lb::Vec3* inDir, lb::Vec3* outDir)
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
        float phi   = ss2->getTheta(i1);

        *inDir = lb::SphericalCoordinateSystem::toXyz(theta, phi);
        *outDir = lb::reflect(*inDir, lb::Vec3(0, 0, 1));
    }
    else {
        return false;
    }

    return true;
}

void TableView::addAngleItem(const QColor& color, float angle,
                             qreal posX, qreal posY,
                             qreal lodThreshold, qreal textLodThreshold)
{
    GraphicsAngleItem* item = new GraphicsAngleItem(color, angle);
    item->setPos(posX, posY);
    item->setLodThreshold(lodThreshold);
    item->setTextLodThreshold(textLodThreshold);
    tableScene_->addItem(item);
}

void TableView::paintEvent(QPaintEvent* event)
{
    QGraphicsView::paintEvent(event);

    const lb::Brdf* brdf = data_->getBrdfData();
    const lb::SampleSet2D* ss2 = 0;
    if (!brdf) {
        if (data_->getSpecularReflectances()) {
            ss2 = data_->getSpecularReflectances().get();
        }
        else if (data_->getSpecularTransmittances()) {
            ss2 = data_->getSpecularTransmittances().get();
        }
        else {
            return;
        }
    }

    QPainter painter(viewport());

    painter.save();

    QFont font("Helvetica");
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
        painter.drawText(angleNameRect, Qt::AlignCenter, util::toSentenceCase(brdf->getAngle0Name().c_str()));

        painter.translate(0, angleNameRect.height());
        painter.fillRect(angleNameRect, QColor(Qt::green).lighter(180));
        painter.drawRect(angleNameRect);
        painter.drawText(angleNameRect, Qt::AlignCenter, util::toSentenceCase(brdf->getAngle2Name().c_str()));

        painter.resetTransform();
        painter.rotate(-90.0);

        if (numAngle1 == 1) {
            painter.translate(-angleNameRect.width() - angleNameRect.height() * 2 - 1, -angleNameRect.height() + 1);
        }
        else {
            painter.translate(-angleNameRect.width() - angleNameRect.height() * 2 - 1, 1);
            painter.fillRect(angleNameRect, QColor(Qt::yellow).lighter(180));
            painter.drawRect(angleNameRect);
            painter.drawText(angleNameRect, Qt::AlignCenter, util::toSentenceCase(brdf->getAngle1Name().c_str()));
        }

        painter.translate(0, angleNameRect.height());
        painter.fillRect(angleNameRect, QColor(Qt::blue).lighter(180));
        painter.drawRect(angleNameRect);
        painter.drawText(angleNameRect, Qt::AlignCenter, util::toSentenceCase(brdf->getAngle3Name().c_str()));
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
    actionShowBackSide_->setChecked(backSideShown_);

    QMenu menu(this);
    menu.addAction(actionFitView_);
    menu.addSeparator();
    menu.addAction(actionLogPlot_);
    menu.addAction(actionShowBackSide_);
    menu.exec(event->globalPos());
}

void TableView::showEvent(QShowEvent* event)
{
    Q_UNUSED(event);

    // Initialize the scale of this view with scene.
    // If data is set before the view is shown, the proper size of the view is not acquired.
    if (fittingNeeded_) {
        fitView(0.9);
    }
    fittingNeeded_ = false;
}
