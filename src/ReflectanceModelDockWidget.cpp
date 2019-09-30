// =================================================================== //
// Copyright (C) 2016-2019 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "ReflectanceModelDockWidget.h"

#include <osg/Timer>

#include <libbsdf/Brdf/HalfDifferenceCoordinatesBrdf.h>
#include <libbsdf/Brdf/SpecularCoordinatesBrdf.h>
#include <libbsdf/Brdf/SphericalCoordinatesBrdf.h>

#include <libbsdf/Common/Log.h>

#include <libbsdf/ReflectanceModel/AshikhminShirley.h>
#include <libbsdf/ReflectanceModel/BlinnPhong.h>
#include <libbsdf/ReflectanceModel/CookTorrance.h>
#include <libbsdf/ReflectanceModel/Disney.h>
#include <libbsdf/ReflectanceModel/GGX.h>
#include <libbsdf/ReflectanceModel/GgxAnisotropic.h>
#include <libbsdf/ReflectanceModel/Lambertian.h>
#include <libbsdf/ReflectanceModel/Minnaert.h>
#include <libbsdf/ReflectanceModel/ModifiedPhong.h>
#include <libbsdf/ReflectanceModel/MultipleScatteringSmith.h>
#include <libbsdf/ReflectanceModel/OrenNayar.h>
#include <libbsdf/ReflectanceModel/Phong.h>
#include <libbsdf/ReflectanceModel/ReflectanceModelUtility.h>
#include <libbsdf/ReflectanceModel/SimplifiedOrenNayar.h>
#include <libbsdf/ReflectanceModel/WardAnisotropic.h>
#include <libbsdf/ReflectanceModel/WardIsotropic.h>

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
    lb::reflectance_model_utility::setupTabularBrdf(*model, brdf.get());

    osg::Timer_t endTick = osg::Timer::instance()->tick();
    double delta = osg::Timer::instance()->delta_s(startTick, endTick);
    lbInfo << "[ReflectanceModelDockWidget::generateBrdf] " << delta << "(s)";

    emit generated(brdf, lb::BRDF_DATA);
    emit generated();
}

void ReflectanceModelDockWidget::initializeReflectanceModels()
{
    std::vector<lb::ReflectanceModel*> models;

    lb::Vec3 white(1.0, 1.0, 1.0);
    lb::Vec3 black(0.0, 0.0, 0.0);
    models.push_back(new lb::AshikhminShirley(white, black, 100.0f, 20.0f));
    models.push_back(new lb::BlinnPhong(white, 40.0f));
    models.push_back(new lb::CookTorrance(white, 0.3f));
    models.push_back(new lb::Disney(white, black, 0.2f, 0.4f));
    models.push_back(new lb::Ggx(white, 0.3f));
    models.push_back(new lb::GgxAnisotropic(white, 0.2f, 0.4f));
    models.push_back(new lb::Lambertian(white));
    models.push_back(new lb::Minnaert(white, 0.83f));
    models.push_back(new lb::ModifiedPhong(white, 10.0f));
    models.push_back(new lb::MultipleScatteringSmith(white, 0.2f, 0.4f, 1.5f,
                                                     int(lb::MultipleScatteringSmith::DIELECTRIC_MATERIAL),
                                                     int(lb::MultipleScatteringSmith::GAUSSIAN_HEIGHT),
                                                     int(lb::MultipleScatteringSmith::BECKMANN_SLOPE),
                                                     10));
    models.push_back(new lb::OrenNayar(white, 0.3f));
    models.push_back(new lb::Phong(white, 10.0f));
    models.push_back(new lb::SimplifiedOrenNayar(white, 0.3f));
    models.push_back(new lb::WardAnisotropic(white, 0.05f, 0.2f));
    models.push_back(new lb::WardIsotropic(white, 0.2f));

    for (auto it = models.begin(); it != models.end(); ++it) {
        reflectanceModels_[(*it)->getName()] = *it;
    }

    AnalyticBsdfDockWidget::initializeReflectanceModels();
}
