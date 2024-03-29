// =================================================================== //
// Copyright (C) 2016-2023 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "ReflectanceModelDockWidget.h"

#include <osg/Timer>

#include <libbsdf/Brdf/HalfDifferenceCoordinatesBrdf.h>
#include <libbsdf/Brdf/Optimizer.h>
#include <libbsdf/Brdf/SpecularCoordinatesBrdf.h>
#include <libbsdf/Brdf/SphericalCoordinatesBrdf.h>

#include <libbsdf/ReflectanceModel/AnisotropicGgx.h>
#include <libbsdf/ReflectanceModel/AnisotropicWard.h>
#include <libbsdf/ReflectanceModel/AshikhminShirley.h>
#include <libbsdf/ReflectanceModel/BlinnPhong.h>
#include <libbsdf/ReflectanceModel/CookTorrance.h>
#include <libbsdf/ReflectanceModel/Disney.h>
#include <libbsdf/ReflectanceModel/Ggx.h>
#include <libbsdf/ReflectanceModel/IsotropicWard.h>
#include <libbsdf/ReflectanceModel/Lambertian.h>
#include <libbsdf/ReflectanceModel/Minnaert.h>
#include <libbsdf/ReflectanceModel/ModifiedPhong.h>
#include <libbsdf/ReflectanceModel/MultipleScatteringSmith.h>
#include <libbsdf/ReflectanceModel/OrenNayar.h>
#include <libbsdf/ReflectanceModel/Phong.h>
#include <libbsdf/ReflectanceModel/ReflectanceModelUtility.h>
#include <libbsdf/ReflectanceModel/SimpleAnisotropicGgx.h>
#include <libbsdf/ReflectanceModel/SimpleGgx.h>
#include <libbsdf/ReflectanceModel/SimplifiedOrenNayar.h>
#include <libbsdf/ReflectanceModel/UnrealEngine4.h>

ReflectanceModelDockWidget::ReflectanceModelDockWidget(QWidget* parent)
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

void ReflectanceModelDockWidget::generateBrdf()
{
    lbTrace << "[ReflectanceModelDockWidget::generateBrdf]";

    osg::Timer_t startTick = osg::Timer::instance()->tick();

    QString name = ui_->reflectanceModelComboBox->currentText();
    lb::ReflectanceModel* model = reflectanceModels_[name.toLocal8Bit().data()];

    std::shared_ptr<lb::Brdf> brdf = initializeBrdf(model->isIsotropic());
    lb::ReflectanceModelUtility::setupBrdf(*model, brdf.get());

    if (ui_->intervalAdjustmentCheckBox->isChecked()) {
        const lb::SampleSet* ss = brdf->getSampleSet();

        // Save the number of angles before optimization.
        int numAngles0 = ss->getNumAngles0();
        int numAngles1 = ss->getNumAngles1();
        int numAngles2 = ss->getNumAngles2();
        int numAngles3 = ss->getNumAngles3();

        lb::Optimizer optimizer(brdf.get(), 0.001, 0.01);
        optimizer.optimize();

        lb::ReflectanceModelUtility::setupBrdf(*model, brdf.get(),
                                               numAngles0, numAngles1, numAngles2, numAngles3);
    }

    osg::Timer_t endTick = osg::Timer::instance()->tick();
    double delta = osg::Timer::instance()->delta_s(startTick, endTick);
    lbInfo << "[ReflectanceModelDockWidget::generateBrdf] " << delta << "(s)";

    emit generated(brdf, lb::BRDF_DATA);
    emit generated();
}

void ReflectanceModelDockWidget::initializeReflectanceModels()
{
    std::vector<lb::ReflectanceModel*> models;

    const lb::Vec3 white(1.0, 1.0, 1.0);
    const lb::Vec3 gray5(0.05, 0.05, 0.05);
    const lb::Vec3 gray90(0.9, 0.9, 0.9);
    const lb::Vec3 black(0.0, 0.0, 0.0);

    // The refractive index and extinction coefficient of aluminium at 550nm.
    constexpr double n = 0.96521;
    constexpr double k = 6.3995;

    models.push_back(new lb::AshikhminShirley(white, black, 100.0, 20.0));
    models.push_back(new lb::BlinnPhong(white, 40.0));
    models.push_back(new lb::CookTorrance(white, 0.3));
    models.push_back(new lb::Disney(white, black, 0.2, 0.4));
    models.push_back(new lb::Ggx(white, 0.3, 1.5, 0.0));
    models.push_back(new lb::SimpleGgx(gray5, 0.3));
    models.push_back(new lb::AnisotropicGgx(white, 0.2, 0.4, n, k));
    models.push_back(new lb::SimpleAnisotropicGgx(gray90, 0.2, 0.4));
    models.push_back(new lb::Lambertian(white));
    models.push_back(new lb::Minnaert(white, 0.83));
    models.push_back(new lb::ModifiedPhong(white, 10.0));
    models.push_back(new lb::MultipleScatteringSmith(white, 0.2, 0.4, 1.5,
                                                     int(lb::MultipleScatteringSmith::DIELECTRIC_MATERIAL),
                                                     int(lb::MultipleScatteringSmith::GAUSSIAN_HEIGHT),
                                                     int(lb::MultipleScatteringSmith::BECKMANN_SLOPE),
                                                     10));
    models.push_back(new lb::OrenNayar(white, 0.3));
    models.push_back(new lb::Phong(white, 10.0));
    models.push_back(new lb::SimplifiedOrenNayar(white, 0.3));
    models.push_back(new lb::UnrealEngine4(white, 0.0, 0.5, 0.3));
    models.push_back(new lb::AnisotropicWard(white, 0.05, 0.2));
    models.push_back(new lb::IsotropicWard(white, 0.2));

    for (auto it = models.begin(); it != models.end(); ++it) {
        reflectanceModels_[(*it)->getName()] = *it;
    }

    AnalyticBsdfDockWidget::initializeReflectanceModels();
}
