// =================================================================== //
// Copyright (C) 2020 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "PickDockWidget.h"

#include <QtWidgets>

#include <osg/io_utils>

#include <libbsdf/Common/SpectrumUtility.h>

const QString redStyleSheet     = "QLabel { background-color: rgb(255, 190, 190); }";
const QString yellowStyleSheet  = "QLabel { background-color: rgb(245, 245, 180); }";
const QString greenStyleSheet   = "QLabel { background-color: rgb(190, 255, 190); }";
const QString blueStyleSheet    = "QLabel { background-color: rgb(190, 190, 255); }";

PickDockWidget::PickDockWidget(QWidget* parent)
                               : QDockWidget(parent),
                                 graphScene_(nullptr),
                                 data_(nullptr),
                                 ui_(new Ui::PickDockWidgetBase)
{
    ui_->setupUi(this);

    // Use a spherical coordinate system as default.
    ui_->coordSysComboBox->setCurrentIndex(2);
    ui_->coordSysComboBox->activated(2);

    actionCopyInfo_ = new QAction(this);
    actionCopyInfo_->setText("Copy values");
    connect(actionCopyInfo_, SIGNAL(triggered()), this, SLOT(copyInfo()));
}

PickDockWidget::~PickDockWidget()
{
    delete ui_;
}

void PickDockWidget::updatePickedAngle(const lb::Vec3& inDir, const lb::Vec3& outDir, bool isotropic)
{
    auto toNumber = [](double value) {
        if (std::abs(value) < 0.000001) {
            value = 0.0;
        }
        return QString::number(lb::toDegree(value));
    };

    // Set angles of lb::HalfDifferenceCoordinateSystem.
    {
        float halfTheta, halfPhi, diffTheta, diffPhi;
        data_->getHalfDiffCoordAngles(inDir, outDir, &halfTheta, &halfPhi, &diffTheta, &diffPhi);

        ui_->halfDiffCsAngle0LineEdit->setText(toNumber(halfTheta));
        ui_->halfDiffCsAngle1LineEdit->setText(toNumber(halfPhi));
        ui_->halfDiffCsAngle2LineEdit->setText(toNumber(diffTheta));
        ui_->halfDiffCsAngle3LineEdit->setText(toNumber(diffPhi));
    }

    // Set angles of lb::SpecularCoordinateSystem.
    {
        float inTheta, inPhi, specTheta, specPhi;
        data_->getSpecularCoordAngles(inDir, outDir, &inTheta, &inPhi, &specTheta, &specPhi);

        ui_->specularCsAngle0LineEdit->setText(toNumber(inTheta));
        ui_->specularCsAngle1LineEdit->setText(toNumber(inPhi));
        ui_->specularCsAngle2LineEdit->setText(toNumber(specTheta));
        ui_->specularCsAngle3LineEdit->setText(toNumber(specPhi));
    }

    // Set angles of lb::SphericalCoordinateSystem.
    {
        float inTheta, inPhi, outTheta, outPhi;
        data_->getShericalCoordAngles(inDir, outDir, &inTheta, &inPhi, &outTheta, &outPhi);

        ui_->sphericalCsAngle0LineEdit->setText(toNumber(inTheta));
        ui_->sphericalCsAngle1LineEdit->setText(toNumber(inPhi));
        ui_->sphericalCsAngle2LineEdit->setText(toNumber(outTheta));
        ui_->sphericalCsAngle3LineEdit->setText(toNumber(outPhi));
    }
}

void PickDockWidget::updatePickedValue(const lb::Vec3& inDir, const lb::Vec3& outDir)
{
    // Clear values if the picked direction in the other view is empty.
    if (outDir.isZero()) {
        clearPickedValue();
        return;
    }

    bool isotropic;
    lb::Spectrum sp;
    lb::Arrayf wls;
    lb::ColorModel cm;

    if (lb::Brdf* brdf = data_->getBrdf().get()) {
        isotropic = brdf->getSampleSet()->isIsotropic();
        sp = brdf->getSpectrum(inDir, outDir);
        wls = brdf->getSampleSet()->getWavelengths();
        cm = brdf->getSampleSet()->getColorModel();
    }
    else if (lb::SampleSet2D* sr = data_->getSpecularReflectances().get()) {
        isotropic = sr->isIsotropic();
        sp = sr->getSpectrum(inDir);
        wls = sr->getWavelengths();
        cm = sr->getColorModel();
    }
    else if (lb::Btdf* btdf = data_->getBtdf().get()) {
        isotropic = btdf->getSampleSet()->isIsotropic();
        sp = btdf->getSpectrum(inDir, outDir);
        wls = btdf->getSampleSet()->getWavelengths();
        cm = btdf->getSampleSet()->getColorModel();
    }
    else if (lb::SampleSet2D* st = data_->getSpecularTransmittances().get()) {
        isotropic = st->isIsotropic();
        sp = st->getSpectrum(inDir);
        wls = st->getWavelengths();
        cm = st->getColorModel();
    }
    else {
        return;
    }

    updatePickedAngle(inDir, outDir, isotropic);

    if (ui_->pickedValueLineEdit->isEnabled()) {
        float pickedValue;
        if (graphScene_->getDisplayMode() == GraphScene::PHOTOMETRY_DISPLAY) {
            pickedValue = lb::SpectrumUtility::spectrumToY(sp, cm, wls);
        }
        else {
            pickedValue = sp[graphScene_->getWavelengthIndex()];
        }

        ui_->pickedValueLineEdit->setText(QString::number(pickedValue));
    }
}

void PickDockWidget::updatePickedValue()
{
    updatePickedValue(graphScene_->getPickedInDir(), graphScene_->getPickedOutDir());
}

void PickDockWidget::displayReflectance()
{
    if (!ui_->pickedReflectanceLineEdit->isEnabled()) return;

    lb::SampleSet2D* ss2 = 0;

    if (data_->getBrdf()) {
        ss2 = data_->getReflectances();
        ui_->pickedReflectanceLabel->setText("Reflectance:");
    }
    else if (data_->getBtdf()) {
        ss2 = data_->getReflectances();
        ui_->pickedReflectanceLabel->setText("Transmittance:");
    }
    else if (data_->getSpecularReflectances()) {
        ss2 = data_->getSpecularReflectances().get();
        ui_->pickedReflectanceLabel->setText("Reflectance:");
    }
    else if (data_->getSpecularTransmittances()) {
        ss2 = data_->getSpecularTransmittances().get();
        ui_->pickedReflectanceLabel->setText("Transmittance:");
    }
    else {
        return;
    }

    if (data_->isReflectancesComputed() ||
        data_->getSpecularReflectances() ||
        data_->getSpecularTransmittances()) {
        float reflectance;

        const lb::Spectrum& sp = ss2->getSpectrum(graphScene_->getInTheta(), graphScene_->getInPhi());
        if (graphScene_->getDisplayMode() == GraphScene::PHOTOMETRY_DISPLAY) {
            reflectance = lb::SpectrumUtility::spectrumToY(sp, ss2->getColorModel(), ss2->getWavelengths());
        }
        else {
            reflectance = sp[graphScene_->getWavelengthIndex()];
        }

        ui_->pickedReflectanceLineEdit->setText(QString::number(reflectance));
    }
    else {
        ui_->pickedReflectanceLineEdit->setText("Computing");
    }
}

void PickDockWidget::updateOutDir(const osg::Vec3& dir)
{
    lbDebug << "[PickDockWidget::updateOutDir] position: " << dir;

    if (!ui_->pickedValueLineEdit->isEnabled()) {
        ui_->pickedValueLineEdit->clear();
        return;
    }

    lb::Vec3 inDir = lb::SphericalCoordinateSystem::toXyz(graphScene_->getInTheta(), graphScene_->getInPhi());
    lb::Vec3 outDir = lb::toVec3(dir).normalized();

    graphScene_->updateInOutDirLine(inDir, outDir);
    updatePickedValue();

    emit redrawGraphRequested();
}

void PickDockWidget::clearPickedValue()
{
    ui_->halfDiffCsAngle0LineEdit->clear();
    ui_->halfDiffCsAngle1LineEdit->clear();
    ui_->halfDiffCsAngle2LineEdit->clear();
    ui_->halfDiffCsAngle3LineEdit->clear();

    ui_->specularCsAngle0LineEdit->clear();
    ui_->specularCsAngle1LineEdit->clear();
    ui_->specularCsAngle2LineEdit->clear();
    ui_->specularCsAngle3LineEdit->clear();

    ui_->sphericalCsAngle0LineEdit->clear();
    ui_->sphericalCsAngle1LineEdit->clear();
    ui_->sphericalCsAngle2LineEdit->clear();
    ui_->sphericalCsAngle3LineEdit->clear();

    ui_->pickedValueLineEdit->clear();

    graphScene_->clearOutDirLine();

    emit redrawGraphRequested();
}

void PickDockWidget::displayArcsInGraph(int index)
{
    if (!graphScene_) return;

    GraphScene::ArcDisplayMode mode = static_cast<GraphScene::ArcDisplayMode>(index);
    graphScene_->setArcDisplayMode(mode);
    graphScene_->updateInOutDirLine();

    emit redrawGraphRequested();
}

void PickDockWidget::useArcsInGraph(bool on)
{
    if (!graphScene_) return;

    if (on) {
        ui_->halfDiffCsNumAngle0Label->setStyleSheet(redStyleSheet);
        ui_->halfDiffCsNumAngle1Label->setStyleSheet(yellowStyleSheet);
        ui_->halfDiffCsNumAngle2Label->setStyleSheet(greenStyleSheet);
        ui_->halfDiffCsNumAngle3Label->setStyleSheet(blueStyleSheet);

        ui_->sphericalCsNumAngle0Label->setStyleSheet(redStyleSheet);
        ui_->sphericalCsNumAngle1Label->setStyleSheet(yellowStyleSheet);
        ui_->sphericalCsNumAngle2Label->setStyleSheet(greenStyleSheet);
        ui_->sphericalCsNumAngle3Label->setStyleSheet(blueStyleSheet);

        ui_->specularCsNumAngle0Label->setStyleSheet(redStyleSheet);
        ui_->specularCsNumAngle1Label->setStyleSheet(yellowStyleSheet);
        ui_->specularCsNumAngle2Label->setStyleSheet(greenStyleSheet);
        ui_->specularCsNumAngle3Label->setStyleSheet(blueStyleSheet);
    }
    else {
        ui_->halfDiffCsNumAngle0Label->setStyleSheet("");
        ui_->halfDiffCsNumAngle1Label->setStyleSheet("");
        ui_->halfDiffCsNumAngle2Label->setStyleSheet("");
        ui_->halfDiffCsNumAngle3Label->setStyleSheet("");

        ui_->sphericalCsNumAngle0Label->setStyleSheet("");
        ui_->sphericalCsNumAngle1Label->setStyleSheet("");
        ui_->sphericalCsNumAngle2Label->setStyleSheet("");
        ui_->sphericalCsNumAngle3Label->setStyleSheet("");

        ui_->specularCsNumAngle0Label->setStyleSheet("");
        ui_->specularCsNumAngle1Label->setStyleSheet("");
        ui_->specularCsNumAngle2Label->setStyleSheet("");
        ui_->specularCsNumAngle3Label->setStyleSheet("");
    }

    graphScene_->setArcDisplayUsed(on);
    graphScene_->updateInOutDirLine();

    emit redrawGraphRequested();
}

void PickDockWidget::copyInfo()
{
    QString sphericalCsAngle0 = "INCOMING_POLAR_ANGLE "     + ui_->sphericalCsAngle0LineEdit->text();
    QString sphericalCsAngle1 = "INCOMING_AZIMUTHAL_ANGLE " + ui_->sphericalCsAngle1LineEdit->text();
    QString sphericalCsAngle2 = "OUTGOING_POLAR_ANGLE "     + ui_->sphericalCsAngle2LineEdit->text();
    QString sphericalCsAngle3 = "OUTGOING_AZIMUTHAL_ANGLE " + ui_->sphericalCsAngle3LineEdit->text();

    QString specularCsAngle2 = "SPECULAR_POLAR_ANGLE "      + ui_->specularCsAngle2LineEdit->text();
    QString specularCsAngle3 = "SPECULAR_AZIMUTHAL_ANGLE "  + ui_->specularCsAngle3LineEdit->text();

    QString halfDiffCsAngle0 = "HALF_POLAR_ANGLE "              + ui_->halfDiffCsAngle0LineEdit->text();
    QString halfDiffCsAngle1 = "HALF_AZIMUTHAL_ANGLE "          + ui_->halfDiffCsAngle1LineEdit->text();
    QString halfDiffCsAngle2 = "DIFFERENCE_POLAR_ANGLE "        + ui_->halfDiffCsAngle2LineEdit->text();
    QString halfDiffCsAngle3 = "DIFFERENCE_AZIMUTHAL_ANGLE "    + ui_->halfDiffCsAngle3LineEdit->text();

    QString value = "VALUE " + ui_->pickedValueLineEdit->text();

    QString reflectance;
    if (data_->getBtdf() || data_->getSpecularTransmittances()) {
        reflectance = "TRANSMITTANCE ";
    }
    else {
        reflectance = "REFLECTANCE ";
    }
    reflectance += ui_->pickedReflectanceLineEdit->text();

    qApp->clipboard()->setText(sphericalCsAngle0 + "\n" +
                               sphericalCsAngle1 + "\n" +
                               sphericalCsAngle2 + "\n" +
                               sphericalCsAngle3 + "\n" +
                               specularCsAngle2 + "\n" +
                               specularCsAngle3 + "\n" +
                               halfDiffCsAngle0 + "\n" +
                               halfDiffCsAngle1 + "\n" +
                               halfDiffCsAngle2 + "\n" +
                               halfDiffCsAngle3 + "\n" +
                               value +"\n" +
                               reflectance);
}

void PickDockWidget::contextMenuEvent(QContextMenuEvent* event)
{
    if (data_->isEmpty()) return;

    QMenu menu(this);
    menu.addAction(actionCopyInfo_);
    menu.exec(event->globalPos());
}
