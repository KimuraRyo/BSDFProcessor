// =================================================================== //
// Copyright (C) 2014-2026 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "TableView.h"

#include <libbsdf/Brdf/SphericalCoordinatesBrdf.h>

#include "Utility.h"

TableView::TableView(QWidget* parent)
    : QGraphicsView(parent),
      actionLogPlot_(nullptr),
      data_(nullptr),
      backSideShown_(true),
      fittingNeeded_(true)
{
    actionFitView_ = new QAction(this);
    actionFitView_->setText("Fit in view");
    connect(actionFitView_, SIGNAL(triggered()), this, SLOT(fitView()));

    actionShowBackSide_ = new QAction(this);
    actionShowBackSide_->setText("Show back side samples");
    actionShowBackSide_->setCheckable(true);
    connect(actionShowBackSide_, SIGNAL(triggered()), this, SLOT(changeBackSideVisibility()));

    tableScene_ = new TableScene;
    setScene(tableScene_);

    viewport()->setCursor(Qt::ArrowCursor);

    connect(tableScene_, SIGNAL(mouseMoved(QPointF)), this, SLOT(showToolTip(QPointF)));
    connect(tableScene_, SIGNAL(mouseClicked(QPointF)), this, SLOT(updateInOutDir(QPointF)));
}

void TableView::updateTable(int wavelengthIndex, float gamma, bool photometric)
{
    if (!data_)
        return;

    tableScene_->initTable(wavelengthIndex, gamma, photometric);
}

void TableView::setMaterialData(MaterialData* materialData)
{
    data_ = materialData;
    tableScene_->setMaterialData(materialData);
}

void TableView::showBackSide(bool on)
{
    backSideShown_ = on;
    tableScene_->setBackSideShown(backSideShown_);
}

void TableView::fitView(qreal scaleFactor)
{
    fitInView(scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
    scale(scaleFactor, scaleFactor);
}

void TableView::changeBackSideVisibility()
{
    backSideShown_ = !backSideShown_;
    tableScene_->setBackSideShown(backSideShown_);
    actionShowBackSide_->setChecked(backSideShown_);
    tableScene_->updateTable();

    const lb::Brdf* brdf = data_->getBrdfData();
    if (!brdf || (brdf && dynamic_cast<const lb::SphericalCoordinatesBrdf*>(brdf))) {
        actionShowBackSide_->setDisabled(true);
    }
    else {
        actionShowBackSide_->setEnabled(true);
    }
}

void TableView::showToolTip(const QPointF& pos)
{
    int i0, i1, i2, i3;
    if (!tableScene_->getIndex(pos, &i0, &i1, &i2, &i3)) {
        setToolTip("");
        return;
    }

    float val;
    if (const lb::SampleSet* ss = data_->getSampleSet()) {
        val = tableScene_->getSampleValue(ss->getSpectrum(i0, i1, i2, i3), ss->getColorModel(),
                                          ss->getWavelengths());
    }
    else if (const lb::SampleSet2D* ss2 = data_->getSampleSet2D()) {
        val = tableScene_->getSampleValue(ss2->getSpectrum(i0, i1), ss2->getColorModel(),
                                          ss2->getWavelengths());
    }
    else {
        return;
    }

    setToolTip(QString::number(val));
}

void TableView::updateInOutDir(const QPointF& pos)
{
    lb::Vec3 inDir, outDir;
    if (tableScene_->getInOutDir(pos, &inDir, &outDir)) {
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

void TableView::paintEvent(QPaintEvent* event)
{
    QGraphicsView::paintEvent(event);

    const lb::Brdf*        brdf = data_->getBrdfData();
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

    QFont font = painter.font();
    font.setPixelSize(11);
    painter.setFont(font);

    QRect angleNameRect(0, 0, 150, 18);

    if (brdf) {
        const lb::SampleSet* ss = brdf->getSampleSet();

        int numAngles1 = ss->getNumAngles1();

        if (numAngles1 == 1) {
            painter.translate(angleNameRect.height() + 1, 1);
        }
        else {
            painter.translate(angleNameRect.height() * 2 + 1, 1);
        }

        painter.fillRect(angleNameRect, QColor(Qt::red).lighter(180));
        painter.drawRect(angleNameRect);
        painter.drawText(angleNameRect, Qt::AlignCenter,
                         util::toSentenceCase(brdf->getAngle0Name().c_str()));

        painter.translate(0, angleNameRect.height());
        painter.fillRect(angleNameRect, QColor(Qt::green).lighter(180));
        painter.drawRect(angleNameRect);
        painter.drawText(angleNameRect, Qt::AlignCenter,
                         util::toSentenceCase(brdf->getAngle2Name().c_str()));

        painter.resetTransform();
        painter.rotate(-90.0);

        if (numAngles1 == 1) {
            painter.translate(-angleNameRect.width() - angleNameRect.height() * 2 - 1,
                              -angleNameRect.height() + 1);
        }
        else {
            painter.translate(-angleNameRect.width() - angleNameRect.height() * 2 - 1, 1);
            painter.fillRect(angleNameRect, QColor(Qt::yellow).lighter(180));
            painter.drawRect(angleNameRect);
            painter.drawText(angleNameRect, Qt::AlignCenter,
                             util::toSentenceCase(brdf->getAngle1Name().c_str()));
        }

        painter.translate(0, angleNameRect.height());
        painter.fillRect(angleNameRect, QColor(Qt::blue).lighter(180));
        painter.drawRect(angleNameRect);
        painter.drawText(angleNameRect, Qt::AlignCenter,
                         util::toSentenceCase(brdf->getAngle3Name().c_str()));
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
    if (event->angleDelta().y() > 0) {
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