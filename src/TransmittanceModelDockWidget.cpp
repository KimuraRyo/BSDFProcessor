// =================================================================== //
// Copyright (C) 2018-2022 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "TransmittanceModelDockWidget.h"

#include <osg/Timer>

#include <libbsdf/Brdf/HalfDifferenceCoordinatesBrdf.h>
#include <libbsdf/Brdf/Optimizer.h>
#include <libbsdf/Brdf/SpecularCoordinatesBrdf.h>
#include <libbsdf/Brdf/SphericalCoordinatesBrdf.h>

#include <libbsdf/ReflectanceModel/AnisotropicGgx.h>
#include <libbsdf/ReflectanceModel/Ggx.h>
#include <libbsdf/ReflectanceModel/Lambertian.h>
#include <libbsdf/ReflectanceModel/MultipleScatteringSmith.h>
#include <libbsdf/ReflectanceModel/ReflectanceModelUtility.h>

TransmittanceModelDockWidget::TransmittanceModelDockWidget(QWidget* parent)
                                                           : AnalyticBsdfDockWidget(parent)
{
    initializeReflectanceModels();
    updateParameterWidget(0);

    int lambertIndex = ui_->reflectanceModelComboBox->findText("Lambertian");
    ui_->reflectanceModelComboBox->setCurrentIndex(lambertIndex);
    ui_->reflectanceModelComboBox->activated(lambertIndex);

    // Select a specular coordinate system.
    ui_->coordSysComboBox->setCurrentIndex(1);
    ui_->coordSysComboBox->activated(1);

    // Increase the number of angles for refraction.
    ui_->halfDiffCsNumAngle2SpinBox->setValue(ui_->halfDiffCsNumAngle2SpinBox->value() * 2);
    ui_->halfDiffCsNumAngle3SpinBox->setValue(ui_->halfDiffCsNumAngle3SpinBox->value() * 2);
    ui_->specularCsNumAngle2SpinBox->setValue(ui_->specularCsNumAngle2SpinBox->value() * 2);

    connect(ui_->generateBrdfPushButton, SIGNAL(clicked()), this, SLOT(generateBrdf()));
}

void TransmittanceModelDockWidget::generateBrdf()
{
    lbTrace << "[TransmittanceModelDockWidget::generateBrdf]";

    osg::Timer_t startTick = osg::Timer::instance()->tick();

    QString name = ui_->reflectanceModelComboBox->currentText();
    lb::ReflectanceModel* model = reflectanceModels_[name.toLocal8Bit().data()];

    std::shared_ptr<lb::Brdf> brdf = initializeBrdf(model->isIsotropic());

    bool iorUsed = (dynamic_cast<lb::Ggx*>(model) ||
                    dynamic_cast<lb::AnisotropicGgx*>(model) ||
                    dynamic_cast<lb::MultipleScatteringSmith*>(model));

    auto specBrdf = dynamic_cast<lb::SpecularCoordinatesBrdf*>(brdf.get());

    float ior = 1.0f;

    // Offset specular directions for refraction.
    if (iorUsed && specBrdf) {
        bool found = false;

        const std::string iorParamName("Refractive index");

        lb::ReflectanceModel::Parameters& params = model->getParameters();
        for (auto it = params.begin(); it != params.end(); ++it) {
            if (it->getName() == iorParamName) {
                ior = *it->getFloat();
                if (ior == 1.0f) break;

                specBrdf->setupSpecularOffsets(ior);

                found = true;
                break;
            }
        }

        if (!found) {
            lbError << "[TransmittanceModelDockWidget::generateBrdf] \"" << iorParamName << "\" is not found.";
        }
    }

    lb::ReflectanceModelUtility::setupBrdf(*model, brdf.get(), lb::BTDF_DATA);

    if (ui_->intervalAdjustmentCheckBox->isChecked()) {
        const lb::SampleSet* ss = brdf->getSampleSet();

        // Save the number of angles before optimization.
        int numAngles0 = ss->getNumAngles0();
        int numAngles1 = ss->getNumAngles1();
        int numAngles2 = ss->getNumAngles2();
        int numAngles3 = ss->getNumAngles3();

        lb::Optimizer optimizer(brdf.get(), 0.001f, 0.01f);
        optimizer.optimize();

        lb::ReflectanceModelUtility::setupBrdf(*model, brdf.get(),
                                               numAngles0, numAngles1, numAngles2, numAngles3,
                                               lb::BTDF_DATA, ior);
    }

    osg::Timer_t endTick = osg::Timer::instance()->tick();
    double delta = osg::Timer::instance()->delta_s(startTick, endTick);
    lbInfo << "[TransmittanceModelDockWidget::generateBrdf] " << delta << "(s)";

    emit generated(brdf, lb::BTDF_DATA);
    emit generated();
}

void TransmittanceModelDockWidget::initializeReflectanceModels()
{
    std::vector<lb::ReflectanceModel*> models;

    lb::Vec3 white(1.0, 1.0, 1.0);
    lb::Vec3 black(0.0, 0.0, 0.0);
    models.push_back(new lb::Ggx(white, 0.3f, 1.5f, 0.0f));
    models.push_back(new lb::AnisotropicGgx(white, 0.2f, 0.4f, 1.5f, 0.0f));
    models.push_back(new lb::Lambertian(white));
    models.push_back(new lb::MultipleScatteringSmith(white, 0.2f, 0.4f, 1.5f,
                                                     int(lb::MultipleScatteringSmith::DIELECTRIC_MATERIAL),
                                                     int(lb::MultipleScatteringSmith::GAUSSIAN_HEIGHT),
                                                     int(lb::MultipleScatteringSmith::BECKMANN_SLOPE),
                                                     10));

    for (auto it = models.begin(); it != models.end(); ++it) {
        reflectanceModels_[(*it)->getName()] = *it;
    }

    AnalyticBsdfDockWidget::initializeReflectanceModels();
}
