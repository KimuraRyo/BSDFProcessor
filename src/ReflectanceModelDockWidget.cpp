// =================================================================== //
// Copyright (C) 2016 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "ReflectanceModelDockWidget.h"

#include <iostream>

#include <osg/Timer>

#include <libbsdf/Brdf/HalfDifferenceCoordinatesBrdf.h>
#include <libbsdf/Brdf/SpecularCoordinatesBrdf.h>
#include <libbsdf/Brdf/SphericalCoordinatesBrdf.h>

#include <libbsdf/ReflectanceModel/BlinnPhong.h>
#include <libbsdf/ReflectanceModel/CookTorrance.h>
#include <libbsdf/ReflectanceModel/Lambertian.h>
#include <libbsdf/ReflectanceModel/ModifiedPhong.h>
#include <libbsdf/ReflectanceModel/Phong.h>
#include <libbsdf/ReflectanceModel/ReflectanceModelUtility.h>
#include <libbsdf/ReflectanceModel/WardAnisotropic.h>
#include <libbsdf/ReflectanceModel/WardIsotropic.h>

#include "QtOsgUtil.h"

ReflectanceModelDockWidget::ReflectanceModelDockWidget(QWidget* parent)
                                                       : QDockWidget(parent),
                                                         ui_(new Ui::ReflectanceModelDockWidgetBase)
{
    ui_->setupUi(this);

    initializeReflectanceModels();
    initializeColorButton();
    updateParameterWidget(0);

    int lambertIndex = ui_->reflectanceModelComboBox->findText("Lambertian");
    ui_->reflectanceModelComboBox->setCurrentIndex(lambertIndex);
    ui_->reflectanceModelComboBox->activated(lambertIndex);

    // Select a specular coordinate system.
    ui_->coordSysComboBox->setCurrentIndex(1);
    ui_->coordSysComboBox->activated(1);

    connect(ui_->generateBrdfPushButton, SIGNAL(clicked()), this, SLOT(generateBrdf()));
}

ReflectanceModelDockWidget::~ReflectanceModelDockWidget()
{
    for (auto it = reflectanceModels_.begin(); it != reflectanceModels_.end(); ++it) {
        delete it->second;
    }

    delete ui_;
}

void ReflectanceModelDockWidget::updateParameterWidget(int index)
{
    qt_osg_util::setBackgroundColor(colorPushButton_, colors_.at(index));    
    
    // Remove items without the label and field of color.
    while (ui_->parameterFormLayout->count() >= 3) {
        QLayoutItem* item = ui_->parameterFormLayout->takeAt(2);
        delete item->widget();
        delete item;
    }

    currentParameters_.clear();

    // Add labels and fields.
    QString name = ui_->reflectanceModelComboBox->itemText(index);
    lb::ReflectanceModel* model = reflectanceModels_[name.toLocal8Bit().data()];
    lb::ReflectanceModel::Parameters& params = model->getParameters();
    for (auto it = params.begin(); it != params.end(); ++it) {
        QDoubleSpinBox* valueSpinBox = new QDoubleSpinBox(ui_->parameterWidget);
        valueSpinBox->setDecimals(3);
        valueSpinBox->setMinimum(0.001);
        valueSpinBox->setMaximum(999.999);
        valueSpinBox->setSingleStep(0.01);
        valueSpinBox->setValue(*it->second);
        valueSpinBox->setMaximumWidth(70);

        std::string label = it->first + ":";
        ui_->parameterFormLayout->addRow(label.c_str(), valueSpinBox);

        currentParameters_[valueSpinBox] = it->second;

        connect(valueSpinBox, SIGNAL(valueChanged(double)),
                this, SLOT(updateParameter(double)));
    }
}

void ReflectanceModelDockWidget::updateCoordSysWidget(int index)
{
    QString name = ui_->reflectanceModelComboBox->itemText(index);
    lb::ReflectanceModel* model = reflectanceModels_[name.toLocal8Bit().data()];

    QLabel* halfLabel = ui_->halfDiffCsNumAngle1Label;
    QLabel* specLabel = ui_->specularCsNumAngle1Label;
    QLabel* spheLabel = ui_->sphericalCsNumAngle1Label;
    QSpinBox* halfSpinBox = ui_->halfDiffCsNumAngle1SpinBox;
    QSpinBox* specSpinBox = ui_->specularCsNumAngle1SpinBox;
    QSpinBox* spheSpinBox = ui_->sphericalCsNumAngle1SpinBox;

    // Hide labels and fields for an anisotropic BRDF if an isotropic reflectance model is selected.
    if (model->isIsotropic()) {
        halfLabel->hide();
        halfSpinBox->hide();
        ui_->halfDiffCoordSysFormLayout->removeWidget(halfLabel);
        ui_->halfDiffCoordSysFormLayout->removeWidget(halfSpinBox);

        specLabel->hide();
        specSpinBox->hide();
        ui_->specularCoordSysFormLayout->removeWidget(specLabel);
        ui_->specularCoordSysFormLayout->removeWidget(specSpinBox);

        spheLabel->hide();
        spheSpinBox->hide();
        ui_->sphericalCoordSysFormLayout->removeWidget(spheLabel);
        ui_->sphericalCoordSysFormLayout->removeWidget(spheSpinBox);
    }
    else {
        ui_->halfDiffCoordSysFormLayout->insertRow(2, halfLabel, halfSpinBox);
        halfLabel->show();
        halfSpinBox->show();

        ui_->specularCoordSysFormLayout->insertRow(2, specLabel, specSpinBox);
        specLabel->show();
        specSpinBox->show();

        ui_->sphericalCoordSysFormLayout->insertRow(2, spheLabel, spheSpinBox);
        spheLabel->show();
        spheSpinBox->show();
    }
}

void ReflectanceModelDockWidget::changeButtonColor()
{
    QColor color = qt_osg_util::setBackgroundColor(colorPushButton_);
    if (!color.isValid()) return;

    int index = ui_->reflectanceModelComboBox->currentIndex();
    colors_.at(index) = color;
}

void ReflectanceModelDockWidget::updateParameter(double value)
{
    for (auto it = currentParameters_.begin(); it != currentParameters_.end(); ++it) {
        *it->second = it->first->value();
    }
}

void ReflectanceModelDockWidget::generateBrdf()
{
    std::cout << "[ReflectanceModelDockWidget::generateBrdf]" << std::endl;

    osg::Timer_t startTick = osg::Timer::instance()->tick();

    QString name = ui_->reflectanceModelComboBox->currentText();
    lb::ReflectanceModel* model = reflectanceModels_[name.toLocal8Bit().data()];
    QColor color = colors_.at(ui_->reflectanceModelComboBox->currentIndex());

    lb::Brdf* brdf = initializeBrdf(model->isIsotropic());
    lb::setupBrdf(*model, brdf, qt_osg_util::qtToLb(color));

    osg::Timer_t endTick = osg::Timer::instance()->tick();
    double delta = osg::Timer::instance()->delta_s(startTick, endTick);
    std::cout << "[ReflectanceModelDockWidget::generateBrdf] " << delta << "(s)" << std::endl;

    emit generated(brdf, lb::BRDF_DATA);
}

void ReflectanceModelDockWidget::initializeReflectanceModels()
{
    std::vector<lb::ReflectanceModel*> models;
    models.push_back(new lb::BlinnPhong(40.0f));
    models.push_back(new lb::CookTorrance(0.2f, 5.0f));
    models.push_back(new lb::Lambertian);
    models.push_back(new lb::ModifiedPhong(10.0f));
    models.push_back(new lb::Phong(10.0f));
    models.push_back(new lb::WardAnisotropic(0.05f, 0.2f));
    models.push_back(new lb::WardIsotropic(0.2f));

    for (auto it = models.begin(); it != models.end(); ++it) {
        reflectanceModels_[(*it)->getName()] = *it;
    }

    for (auto it = reflectanceModels_.begin(); it != reflectanceModels_.end(); ++it) {
        ui_->reflectanceModelComboBox->addItem(it->first.c_str());
        colors_.push_back(Qt::white);
    }

    connect(ui_->reflectanceModelComboBox, SIGNAL(activated(int)),
            this, SLOT(updateParameterWidget(int)));

    connect(ui_->reflectanceModelComboBox, SIGNAL(activated(int)),
            this, SLOT(updateCoordSysWidget(int)));
}

void ReflectanceModelDockWidget::initializeColorButton()
{
    colorPushButton_ = new QPushButton(ui_->parameterWidget);
    colorPushButton_->setAutoFillBackground(true);
    colorPushButton_->setFlat(false);
    colorPushButton_->setMaximumWidth(40);
    qt_osg_util::setBackgroundColor(colorPushButton_, QColor(Qt::white));

    ui_->parameterFormLayout->addRow("Color:", colorPushButton_);

    connect(colorPushButton_, SIGNAL(clicked()), this, SLOT(changeButtonColor()));
}

lb::Brdf* ReflectanceModelDockWidget::initializeBrdf(bool isotropic)
{
    lb::Brdf* brdf;
    std::string coordinateSystemName(ui_->coordSysComboBox->currentText().toLocal8Bit());
    if (coordinateSystemName == "Half difference") {
        int numAngle1 = isotropic ? 1 : (ui_->halfDiffCsNumAngle1SpinBox->value() + 1);
        brdf = new lb::HalfDifferenceCoordinatesBrdf(ui_->halfDiffCsNumAngle0SpinBox->value(),
                                                     numAngle1,
                                                     ui_->halfDiffCsNumAngle2SpinBox->value(),
                                                     ui_->halfDiffCsNumAngle3SpinBox->value() + 1,
                                                     lb::RGB_MODEL, 3, true);
    }
    else if (coordinateSystemName == "Specular") {
        int numAngle1 = isotropic ? 1 : (ui_->specularCsNumAngle1SpinBox->value() + 1);
        brdf = new lb::SpecularCoordinatesBrdf(ui_->specularCsNumAngle0SpinBox->value(),
                                               numAngle1,
                                               ui_->specularCsNumAngle2SpinBox->value(),
                                               ui_->specularCsNumAngle3SpinBox->value() + 1,
                                               lb::RGB_MODEL, 3, true);
    }
    else if (coordinateSystemName == "Spherical") {
        int numAngle1 = isotropic ? 1 : (ui_->sphericalCsNumAngle1SpinBox->value() + 1);
        brdf = new lb::SphericalCoordinatesBrdf(ui_->sphericalCsNumAngle0SpinBox->value(),
                                                numAngle1,
                                                ui_->sphericalCsNumAngle2SpinBox->value(),
                                                ui_->sphericalCsNumAngle3SpinBox->value() + 1,
                                                lb::RGB_MODEL, 3, true);
    }
    else {
        std::cout
            << "[ReflectanceModelDockWidget::initializeBrdf] Invalid coordinate system: "
            << coordinateSystemName << std::endl;
        return 0;
    }

    return brdf;
}
