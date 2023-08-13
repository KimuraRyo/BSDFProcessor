// =================================================================== //
// Copyright (C) 2020-2023 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "CharacteristicDockWidget.h"

#include <QtWidgets>

#include <libbsdf/Brdf/Analyzer.h>
#include <libbsdf/Common/SpectrumUtility.h>

#include "Utility.h"

CharacteristicDockWidget::CharacteristicDockWidget(QWidget* parent)
                                                   : QDockWidget(parent),
                                                     data_(nullptr),
                                                     ui_(new Ui::CharacteristicDockWidgetBase)
{
    ui_->setupUi(this);
}

CharacteristicDockWidget::~CharacteristicDockWidget()
{
    delete ui_;
}

void CharacteristicDockWidget::updateData(const MaterialData& materialData)
{
    if (!&materialData)
        return;

    data_ = &materialData;

    ui_->characteristicTreeWidget->clear();

    addReflectanceItems();
    add8DegreeReflectanceItems();

    if (const lb::Brdf* brdf = data_->getBrdfData()) {
        addBihemisphericalReflectanceItems(*brdf);
        addReciprocityItems(*brdf);
    }

    ui_->characteristicTreeWidget->expandAll();

    updateColumnDisplayMode();
}

void CharacteristicDockWidget::resizeEvent(QResizeEvent* event)
{
    QDockWidget::resizeEvent(event);

    updateColumnDisplayMode();
}

void CharacteristicDockWidget::updateColumnDisplayMode()
{
    QTreeWidget* treeWidget = ui_->characteristicTreeWidget;

    treeWidget->header()->setStretchLastSection(false);
    treeWidget->header()->setSectionResizeMode(QHeaderView::ResizeToContents);

    if (treeWidget->width() > treeWidget->columnWidth(0) + treeWidget->columnWidth(1)) {
        treeWidget->header()->setStretchLastSection(true);
    }

    treeWidget->header()->setSectionResizeMode(QHeaderView::Interactive);

    constexpr int borderWidthOffset = 10;
    treeWidget->setColumnWidth(0, treeWidget->columnWidth(0) + borderWidthOffset);

    if (!treeWidget->header()->stretchLastSection()) {
        treeWidget->setColumnWidth(1, treeWidget->columnWidth(1) + borderWidthOffset);
    }
}

void CharacteristicDockWidget::addReflectanceItems()
{
    QTreeWidgetItem* reflectanceItem = new QTreeWidgetItem(ui_->characteristicTreeWidget);

    QString colorSpaceName;
    switch (data_->getColorModel()) {
        case lb::XYZ_MODEL:
            colorSpaceName = " (CIE XYZ)";
            break;
        case lb::RGB_MODEL:
            colorSpaceName = " (RGB)";
            break;
        case lb::SPECTRAL_MODEL: 
            colorSpaceName = " (Spectrum)";
            break;
        default:
            break;
    }

    switch (data_->getDataType()) {
        case lb::BRDF_DATA:
            reflectanceItem->setToolTip(0, "Directional hemispherical reflectance");
        case lb::SPECULAR_REFLECTANCE_DATA:
            reflectanceItem->setText(0, "Reflectance" + colorSpaceName);
            break;
        case lb::BTDF_DATA:
            reflectanceItem->setToolTip(0, "Directional hemispherical transmittance");
        case lb::SPECULAR_TRANSMITTANCE_DATA:
            reflectanceItem->setText(0, "Transmittance" + colorSpaceName);
            break;
        default:
            return;
    }

    QTreeWidgetItem* parentItem = reflectanceItem;
    for (int phIndex = 0; phIndex < data_->getNumInPhi(); ++phIndex) {
        if (data_->getNumInPhi() > 1) {
            double inPhi = data_->getIncomingAzimuthalAngle(phIndex);

            parentItem = new QTreeWidgetItem(reflectanceItem);
            parentItem->setText(0, "Incoming azimuthal angle: " + QString::number(lb::toDegree(inPhi)) + u8"°");
        }

        QTreeWidgetItem* inThetaItem = new QTreeWidgetItem(parentItem);
        inThetaItem->setText(0, "Incoming polar angle:");

        for (int thIndex = 0; thIndex < data_->getNumInTheta(); ++thIndex) {
            double inTheta = data_->getIncomingPolarAngle(thIndex);

            const lb::Spectrum& sp = data_->getReflectances()->getSpectrum(thIndex, phIndex);

            QTreeWidgetItem* item = new QTreeWidgetItem(inThetaItem);
            item->setText(0, QString::number(lb::toDegree(inTheta)) + u8"°");
            item->setText(1, util::arrayToString(sp).c_str());

            setColorIcon(item, sp, data_->getColorModel(), data_->getWavelengths());

            lb::Vec3 xyz = lb::SpectrumUtility::spectrumToXyz(sp, data_->getColorModel(), data_->getWavelengths());
            lb::Vec3 rgb = lb::xyzToSrgb(xyz).cwiseMax(0.0);
            item->setToolTip(1, "Linear sRGB: " + QString(util::arrayToString(rgb).c_str()));
        }
    }
}

void CharacteristicDockWidget::add8DegreeReflectanceItems()
{
    QTreeWidgetItem* reflectanceItem = new QTreeWidgetItem(ui_->characteristicTreeWidget);

    switch (data_->getDataType()) {
        case lb::BRDF_DATA:
        case lb::SPECULAR_REFLECTANCE_DATA:
            reflectanceItem->setText(0, u8"Reflectance at 8°");
            reflectanceItem->setToolTip(0, u8"8°:di (CIE 15, JIS Z 8722)");
            break;
        case lb::BTDF_DATA:
        case lb::SPECULAR_TRANSMITTANCE_DATA:
            reflectanceItem->setText(0, u8"Transmittance at 0°");
            reflectanceItem->setToolTip(0, u8"0°:di (CIE 15, JIS Z 8722)");
            break;
        default:
            return;
    }

    QTreeWidgetItem* parentItem = reflectanceItem;
    for (int phIndex = 0; phIndex < data_->getNumInPhi(); ++phIndex) {
        double inPhi = data_->getIncomingAzimuthalAngle(phIndex);

        if (data_->getNumInPhi() > 1) {
            parentItem = new QTreeWidgetItem(reflectanceItem);
            parentItem->setText(0, "Incoming azimuthal angle: " + QString::number(lb::toDegree(inPhi)) + u8"°");
        }

        lb::Spectrum sp = data_->getReflectances()->getSpectrum(lb::toRadian(8.0f), inPhi);
        addColors(parentItem, sp);
    }
}

void CharacteristicDockWidget::addBihemisphericalReflectanceItems(const lb::Brdf& brdf)
{
    QTreeWidgetItem* reflectanceItem = new QTreeWidgetItem(ui_->characteristicTreeWidget);

    switch (data_->getDataType()) {
        case lb::BRDF_DATA:
            reflectanceItem->setText(0, "Bihemispherical reflectance");
            break;
        case lb::BTDF_DATA:
            reflectanceItem->setText(0, "Bihemispherical transmittance");
            break;
        default:
            return;
    }

    lb::Spectrum sp = lb::computeBihemisphericalReflectance(brdf);
    addColors(reflectanceItem, sp);
}

void CharacteristicDockWidget::addReciprocityItems(const lb::Brdf& brdf)
{
    QTreeWidgetItem* reciprocityItem = new QTreeWidgetItem(ui_->characteristicTreeWidget);

    reciprocityItem->setText(0, "Reciprocity error");
    reciprocityItem->setToolTip(0, "Bihemispherical reflectance of the absolute difference between the original and reversed BRDF");

    lb::Spectrum sp = lb::computeReciprocityError(brdf);
    addColors(reciprocityItem, sp, false, false);
}

void CharacteristicDockWidget::addColors(QTreeWidgetItem*       parentItem,
                                         const lb::Spectrum&    sp,
                                         bool                   labUsed,
                                         bool                   munsellUsed)
{
    lb::ColorModel cm = data_->getColorModel();
    lb::Arrayf wavelengths = data_->getWavelengths();

    QTreeWidgetItem* item = new QTreeWidgetItem(parentItem);
    item->setText(1, util::arrayToString(sp).c_str());

    switch (cm) {
        case lb::MONOCHROMATIC_MODEL:
            item->setText(0, "Value");
            break;
        case lb::XYZ_MODEL: {
            item->setText(0, "CIE XYZ");

            lb::Vec3 xyz(sp[0], sp[1], sp[2]);
            addSrgb(parentItem, xyz);
            addAdobeRgb(parentItem, xyz);

            if (labUsed) {
                addLab(parentItem, xyz);
            }

            if (munsellUsed) {
                addMunsell(parentItem, xyz);
            }
            break;
        }
        case lb::RGB_MODEL:
            item->setText(0, "RGB");
            setColorIcon(item, lb::Vec3(sp[0], sp[1], sp[2]));
            break;
        case lb::SPECTRAL_MODEL: {
            item->setText(0, "Spectrum");
            QString wlStr = "Wavelengths (nm): " + QString(util::arrayToString(wavelengths).c_str());
            item->setToolTip(0, wlStr);
            item->setToolTip(1, wlStr);

            lb::Vec3 xyz = lb::SpectrumUtility::spectrumToXyz(sp, cm, wavelengths);

            addSrgb(parentItem, xyz);
            addAdobeRgb(parentItem, xyz);

            QTreeWidgetItem* xyzItem = new QTreeWidgetItem(parentItem);
            xyzItem->setText(0, "CIE XYZ");
            xyzItem->setText(1, util::arrayToString(xyz).c_str());

            if (labUsed) {
                addLab(parentItem, xyz);
            }

            if (munsellUsed) {
                addMunsell(parentItem, xyz);
            }

            break;
        }
        default:
            break;
    }
}

void CharacteristicDockWidget::addSrgb(QTreeWidgetItem* parentItem, const lb::Vec3& xyz)
{
    lb::Vec3 rgb = lb::xyzToSrgb(xyz).cwiseMax(0.0);
    QTreeWidgetItem* rgbItem = new QTreeWidgetItem(parentItem);
    rgbItem->setText(0, "Linear sRGB");
    rgbItem->setText(1, util::arrayToString(rgb).c_str());
    setColorIcon(rgbItem, rgb);
}

void CharacteristicDockWidget::addAdobeRgb(QTreeWidgetItem* parentItem, const lb::Vec3& xyz)
{
    lb::Vec3 adobeRgb = lb::xyzToAdobeRgb(xyz).cwiseMax(0.0);
    QTreeWidgetItem* adobeRgbItem = new QTreeWidgetItem(parentItem);
    adobeRgbItem->setText(0, "Linear Adobe RGB (1998)");
    adobeRgbItem->setText(1, util::arrayToString(adobeRgb).c_str());
}

void CharacteristicDockWidget::addLab(QTreeWidgetItem* parentItem, const lb::Vec3& xyz)
{
    QTreeWidgetItem* labItem = new QTreeWidgetItem(parentItem);
    lb::Vec3 lab = lb::xyzToLab(xyz);
    labItem->setText(0, "CIE LAB");
    labItem->setText(1, util::arrayToString(lab).c_str());
}

void CharacteristicDockWidget::addMunsell(QTreeWidgetItem* parentItem, const lb::Vec3& xyz)
{
    std::string hue;
    float value;
    int chroma;
    lb::Vec3 munsellXyz = lb::findMunsellProperties(xyz, &hue, &value, &chroma);
    QString munsellStr(hue.c_str());
    if (chroma == 0.0f) {
        munsellStr += QString::number(value);
    }
    else {
        munsellStr += " " + QString::number(value) + "/" + QString::number(chroma);
    }

    QTreeWidgetItem* munsellItem = new QTreeWidgetItem(parentItem);
    munsellItem->setText(0, "Munsell");
    munsellItem->setText(1, munsellStr);

    lb::Vec3 munsellSrgb = lb::xyzToSrgb(munsellXyz).cwiseMax(0.0);
    setColorIcon(munsellItem, munsellSrgb);
    munsellItem->setToolTip(1, "Linear sRGB: " + QString(util::arrayToString(munsellSrgb).c_str()));
}

void CharacteristicDockWidget::setColorIcon(QTreeWidgetItem* item, const lb::Vec3& rgb)
{
    lb::Vec3 iconRgb = util::applyGamma(rgb);
    QColor rgbColor;
    rgbColor.setRgbF(iconRgb[0], iconRgb[1], iconRgb[2]);

    QImage image(32, 32, QImage::Format_RGB888);
    image.fill(rgbColor);

    for (int w = 0; w < image.width();  ++w) {
    for (int h = 0; h < image.height(); ++h) {
        if (w == 0 || w == image.width() - 1 ||
            h == 0 || h == image.height() - 1) {
            image.setPixelColor(w, h, QColor(Qt::black));
        }
        else {
            image.setPixelColor(w, h, rgbColor);
        }
    }}

    item->setIcon(1, QIcon(QPixmap::fromImage(image)));
}

void CharacteristicDockWidget::setColorIcon(QTreeWidgetItem*    item,
                                            const lb::Spectrum& sp,
                                            lb::ColorModel      cm,
                                            const lb::Arrayf&   wavelengths)
{
    lb::Vec3 xyz = lb::SpectrumUtility::spectrumToXyz(sp, cm, wavelengths);
    lb::Vec3 rgb = lb::xyzToSrgb(xyz).cwiseMax(0.0);
    setColorIcon(item, rgb);
}
