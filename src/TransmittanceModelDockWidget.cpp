// =================================================================== //
// Copyright (C) 2018 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "TransmittanceModelDockWidget.h"

#include <iostream>

#include <osg/Timer>

#include <libbsdf/Brdf/HalfDifferenceCoordinatesBrdf.h>
#include <libbsdf/Brdf/SpecularCoordinatesBrdf.h>
#include <libbsdf/Brdf/SphericalCoordinatesBrdf.h>

#include <libbsdf/ReflectanceModel/GGX.h>
#include <libbsdf/ReflectanceModel/GgxAnisotropic.h>
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

    connect(ui_->generateBrdfPushButton, SIGNAL(clicked()), this, SLOT(generateBrdf()));
}

void TransmittanceModelDockWidget::generateBrdf()
{
    std::cout << "[TransmittanceModelDockWidget::generateBrdf]" << std::endl;

    osg::Timer_t startTick = osg::Timer::instance()->tick();

    QString name = ui_->reflectanceModelComboBox->currentText();
    lb::ReflectanceModel* model = reflectanceModels_[name.toLocal8Bit().data()];

    lb::Brdf* brdf = initializeBrdf(model->isIsotropic());

    bool iorUsed = (dynamic_cast<lb::Ggx*>(model) ||
                    dynamic_cast<lb::GgxAnisotropic*>(model) ||
                    dynamic_cast<lb::MultipleScatteringSmith*>(model));

    lb::SpecularCoordinatesBrdf* specBrdf = dynamic_cast<lb::SpecularCoordinatesBrdf*>(brdf);

    // Offset specular directions for refraction.
    if (iorUsed && specBrdf) {
        bool found = false;

        lb::ReflectanceModel::Parameters& params = model->getParameters();
        for (auto it = params.begin(); it != params.end(); ++it) {
            if (it->getName() == "Refractive index") {
                float ior = *it->getFloat();
                for (int i = 0; i < specBrdf->getNumInTheta(); ++i) {
                    float inTheta = specBrdf->getInTheta(i);
                    float sinT = std::min(std::sin(inTheta) / ior, 1.0f);
                    float refractedTheta = std::asin(sinT);
                    specBrdf->setSpecularOffset(i, refractedTheta - inTheta);
                }

                found = true;
                break;
            }
        }

        if (!found) {
            std::cerr
                << "[TransmittanceModelDockWidget::generateBrdf] \"Refractive index\" is not found."
                << std::endl;
        }
    }

    lb::reflectance_model_utility::setupTabularBrdf(*model, brdf, lb::BTDF_DATA);

    osg::Timer_t endTick = osg::Timer::instance()->tick();
    double delta = osg::Timer::instance()->delta_s(startTick, endTick);
    std::cout << "[TransmittanceModelDockWidget::generateBrdf] " << delta << "(s)" << std::endl;

    emit generated(brdf, lb::BTDF_DATA);
    emit generated();
}

void TransmittanceModelDockWidget::initializeReflectanceModels()
{
    std::vector<lb::ReflectanceModel*> models;

    lb::Vec3 white(1.0, 1.0, 1.0);
    lb::Vec3 black(0.0, 0.0, 0.0);
#if !defined(LIBBSDF_USE_COLOR_INSTEAD_OF_REFRACTIVE_INDEX)
    models.push_back(new lb::Ggx(white, 0.3f));
    models.push_back(new lb::GgxAnisotropic(white, 0.2f, 0.4f, 1.5f, 0.0f));
#endif
    models.push_back(new lb::Lambertian(white));
    models.push_back(new lb::MultipleScatteringSmith(white, 0.2f, 0.4f, 1.5f,
                                                     static_cast<int>(lb::MultipleScatteringSmith::DIELECTRIC_MATERIAL),
                                                     static_cast<int>(lb::MultipleScatteringSmith::GAUSSIAN_HEIGHT),
                                                     static_cast<int>(lb::MultipleScatteringSmith::BECKMANN_SLOPE),
                                                     10));

    for (auto it = models.begin(); it != models.end(); ++it) {
        reflectanceModels_[(*it)->getName()] = *it;
    }

    AnalyticBsdfDockWidget::initializeReflectanceModels();
}
