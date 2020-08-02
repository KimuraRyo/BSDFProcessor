// =================================================================== //
// Copyright (C) 2020 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "PropertyDockWidget.h"

#include <QtWidgets>

#include <libbsdf/Brdf/HalfDifferenceCoordinatesBrdf.h>
#include <libbsdf/Brdf/SampleSet2D.h>
#include <libbsdf/Brdf/SpecularCoordinatesBrdf.h>
#include <libbsdf/Brdf/SphericalCoordinatesBrdf.h>

#include "Utility.h"

PropertyDockWidget::PropertyDockWidget(QWidget* parent)
                                       : QDockWidget(parent),
                                         data_(nullptr),
                                         ui_(new Ui::PropertyDockWidgetBase)
{
    ui_->setupUi(this);
}

PropertyDockWidget::~PropertyDockWidget()
{
    delete ui_;
}

void PropertyDockWidget::updateData(const MaterialData& data)
{
    if (!&data) return;

    data_ = &data;

    ui_->propertyTreeWidget->clear();

    addDataTypeItems();
    addColorModelItems();
    addParamTypeItems();
    addSourceTypeItems();

    ui_->propertyTreeWidget->expandAll();

    updateColumnDisplayMode();
}

void PropertyDockWidget::resizeEvent(QResizeEvent* event)
{
    QDockWidget::resizeEvent(event);

    updateColumnDisplayMode();
}

void PropertyDockWidget::updateColumnDisplayMode()
{
    QTreeWidget* treeWidget = ui_->propertyTreeWidget;

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

void PropertyDockWidget::addColorModelItems()
{
    lb::ColorModel cm = data_->getColorModel();

    QString valueStr;
    switch (cm) {
        case lb::MONOCHROMATIC_MODEL:
            valueStr = QString("monochrome");
            break;
        case lb::RGB_MODEL:
            valueStr = QString("RGB");
            break;
        case lb::XYZ_MODEL:
            valueStr = QString("CIE XYZ");
            break;
        case lb::SPECTRAL_MODEL:
            valueStr = QString("spectrum");
            break;
        default:
            break;
    }

    QTreeWidgetItem* item = new QTreeWidgetItem(ui_->propertyTreeWidget);
    item->setText(0, "Color model");
    item->setText(1, valueStr);

    if (cm == lb::SPECTRAL_MODEL) {
        addWavelengthItems(item);
    }
}

void PropertyDockWidget::addDataTypeItems()
{
    QString valueStr;
    switch (data_->getDataType()) {
        case lb::BRDF_DATA:
            valueStr = QString("BRDF");
            break;
        case lb::BTDF_DATA:
            valueStr = QString("BTDF");
            break;
        case lb::SPECULAR_REFLECTANCE_DATA:
            valueStr = QString("specular reflectance");
            break;
        case lb::SPECULAR_TRANSMITTANCE_DATA:
            valueStr = QString("specular transmittance");
            break;
        default:
            break;
    }

    QTreeWidgetItem* item = new QTreeWidgetItem(ui_->propertyTreeWidget);
    item->setText(0, "Data type");
    item->setText(1, valueStr);
}

void PropertyDockWidget::addParamTypeItems()
{
    if (const lb::Brdf* brdf = data_->getBrdfData()) {
        addParamTypeItems(*brdf);
    }
    else if (const lb::SampleSet2D* ss2 = data_->getSampleSet2D()) {
        addParamTypeItems(*ss2);
    }
}

void PropertyDockWidget::addParamTypeItems(const lb::Brdf& brdf)
{
    const lb::SampleSet* ss = brdf.getSampleSet();

    QTreeWidgetItem* parentItem = new QTreeWidgetItem(ui_->propertyTreeWidget);
    parentItem->setText(0, "Parameter");

    QString valueStr;

    if (dynamic_cast<const lb::HalfDifferenceCoordinatesBrdf*>(&brdf)) {
        valueStr = QString("half-difference coordinate system");
    }
    else if (dynamic_cast<const lb::SpecularCoordinatesBrdf*>(&brdf)) {
        valueStr = QString("specular coordinate system");
    }
    else if (dynamic_cast<const lb::SphericalCoordinatesBrdf*>(&brdf)) {
        valueStr = QString("spherical coordinate system");
    }

    QTreeWidgetItem* typeItem = new QTreeWidgetItem(parentItem);
    typeItem->setText(0, "Type");
    typeItem->setText(1, valueStr);

    QTreeWidgetItem* item0 = new QTreeWidgetItem(parentItem);
    QTreeWidgetItem* item1 = new QTreeWidgetItem(parentItem);
    QTreeWidgetItem* item2 = new QTreeWidgetItem(parentItem);
    QTreeWidgetItem* item3 = new QTreeWidgetItem(parentItem);

    item0->setText(0, util::toSentenceCase(brdf.getAngle0Name().c_str()));
    item1->setText(0, util::toSentenceCase(brdf.getAngle1Name().c_str()));
    item2->setText(0, util::toSentenceCase(brdf.getAngle2Name().c_str()));
    item3->setText(0, util::toSentenceCase(brdf.getAngle3Name().c_str()));

    item0->setText(1, util::arrayToString(lb::toDegrees(ss->getAngles0())).c_str());
    item1->setText(1, util::arrayToString(lb::toDegrees(ss->getAngles1())).c_str());
    item2->setText(1, util::arrayToString(lb::toDegrees(ss->getAngles2())).c_str());
    item3->setText(1, util::arrayToString(lb::toDegrees(ss->getAngles3())).c_str());

    QTreeWidgetItem* countItem0 = new QTreeWidgetItem(item0);
    QTreeWidgetItem* countItem1 = new QTreeWidgetItem(item1);
    QTreeWidgetItem* countItem2 = new QTreeWidgetItem(item2);
    QTreeWidgetItem* countItem3 = new QTreeWidgetItem(item3);

    countItem0->setText(0, "Count");
    countItem1->setText(0, "Count");
    countItem2->setText(0, "Count");
    countItem3->setText(0, "Count");

    countItem0->setText(1, QString::number(ss->getAngles0().size()));
    countItem1->setText(1, QString::number(ss->getAngles1().size()));
    countItem2->setText(1, QString::number(ss->getAngles2().size()));
    countItem3->setText(1, QString::number(ss->getAngles3().size()));

    if (auto specBrdf = dynamic_cast<const lb::SpecularCoordinatesBrdf*>(&brdf)) {
        if (specBrdf->getNumSpecularOffsets() > 0) {
            QTreeWidgetItem* item = new QTreeWidgetItem(parentItem);
            item->setText(0, "Specular offset angles");
            item->setText(1, util::arrayToString(lb::toDegrees(specBrdf->getSpecularOffsets())).c_str());

            QTreeWidgetItem* countItem = new QTreeWidgetItem(item);
            countItem->setText(0, "Count");
            countItem->setText(1, QString::number(specBrdf->getSpecularOffsets().size()));
        }
    }
}

void PropertyDockWidget::addParamTypeItems(const lb::SampleSet2D& ss2)
{
    QTreeWidgetItem* item0 = new QTreeWidgetItem(ui_->propertyTreeWidget);
    item0->setText(0, "Incoming polar angle");
    item0->setText(1, util::arrayToString(lb::toDegrees(ss2.getThetaArray())).c_str());

    QTreeWidgetItem* countItem0 = new QTreeWidgetItem(item0);
    countItem0->setText(0, "Count");
    countItem0->setText(1, QString::number(ss2.getThetaArray().size()));

    if (ss2.getNumPhi() > 1) {
        QTreeWidgetItem* item1 = new QTreeWidgetItem(ui_->propertyTreeWidget);
        item1->setText(0, "Incoming azimuthal angle");
        item1->setText(1, util::arrayToString(lb::toDegrees(ss2.getPhiArray())).c_str());

        QTreeWidgetItem* countItem1 = new QTreeWidgetItem(item1);
        countItem1->setText(0, "Count");
        countItem1->setText(1, QString::number(ss2.getPhiArray().size()));
    }
}

void PropertyDockWidget::addSourceTypeItems()
{
    QString valueStr;
    switch (data_->getSourceType()) {
        case lb::EDITED_SOURCE:
            valueStr = QString("edited");
            break;
        case lb::GENERATED_SOURCE:
            valueStr = QString("generated");
            break;
        case lb::MEASURED_SOURCE:
            valueStr = QString("measured");
            break;
        case lb::UNKNOWN_SOURCE:
        default:
            break;
    }

    QTreeWidgetItem* item = new QTreeWidgetItem(ui_->propertyTreeWidget);
    item->setText(0, "Source type");
    item->setText(1, valueStr);
}

void PropertyDockWidget::addWavelengthItems(QTreeWidgetItem* parentItem)
{
    lb::Arrayf wavelengths = data_->getWavelengths();

    QTreeWidgetItem* item = new QTreeWidgetItem(parentItem);
    item->setText(0, "Wavelengths (nm)");
    item->setText(1, util::arrayToString(wavelengths).c_str());

    QTreeWidgetItem* countItem = new QTreeWidgetItem(item);
    countItem->setText(0, "Count");
    countItem->setText(1, QString::number(wavelengths.size()));
}
