// =================================================================== //
// Copyright (C) 2017-2020 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "SmoothDockWidget.h"

#include <libbsdf/Brdf/Smoother.h>

#include "Utility.h"

SmoothDockWidget::SmoothDockWidget(QWidget* parent)
                                   : QDockWidget(parent),
                                     brdf_(0),
                                     ui_(new Ui::SmoothDockWidgetBase)
{
    ui_->setupUi(this);

    connect(ui_->processPushButton, SIGNAL(clicked()), this, SLOT(process()));
}

SmoothDockWidget::~SmoothDockWidget()
{
    delete ui_;
}

void SmoothDockWidget::setBrdf(lb::Brdf* brdf)
{
    brdf_ = brdf;

    ui_->maxIteration0Label->setText(util::toSentenceCase(brdf->getAngle0Name().c_str()) + QString(":"));
    ui_->maxIteration1Label->setText(util::toSentenceCase(brdf->getAngle1Name().c_str()) + QString(":"));
    ui_->maxIteration2Label->setText(util::toSentenceCase(brdf->getAngle2Name().c_str()) + QString(":"));
    ui_->maxIteration3Label->setText(util::toSentenceCase(brdf->getAngle3Name().c_str()) + QString(":"));

    const lb::SampleSet* ss = brdf->getSampleSet();

    if (ss->getNumAngles0() >= 4) {
        ui_->maxIteration0Label->setEnabled(true);
        ui_->maxIteration0SpinBox->setEnabled(true);
    }
    else {
        ui_->maxIteration0Label->setDisabled(true);
        ui_->maxIteration0SpinBox->setDisabled(true);
    }

    if (ss->getNumAngles1() >= 4) {
        ui_->maxIteration1Label->setEnabled(true);
        ui_->maxIteration1SpinBox->setEnabled(true);
    }
    else {
        ui_->maxIteration1Label->setDisabled(true);
        ui_->maxIteration1SpinBox->setDisabled(true);
    }

    if (ss->getNumAngles2() >= 4) {
        ui_->maxIteration2Label->setEnabled(true);
        ui_->maxIteration2SpinBox->setEnabled(true);
    }
    else {
        ui_->maxIteration2Label->setDisabled(true);
        ui_->maxIteration2SpinBox->setDisabled(true);
    }

    if (ss->getNumAngles3() >= 4) {
        ui_->maxIteration3Label->setEnabled(true);
        ui_->maxIteration3SpinBox->setEnabled(true);
    }
    else {
        ui_->maxIteration3Label->setDisabled(true);
        ui_->maxIteration3SpinBox->setDisabled(true);
    }
}

void SmoothDockWidget::process()
{
    if (!brdf_) return;

    lb::Smoother s(brdf_);
    s.setDiffThreshold(ui_->thresholdDoubleSpinBox->value());
    s.setMaxIteration0(ui_->maxIteration0SpinBox->value());
    s.setMaxIteration1(ui_->maxIteration1SpinBox->value());
    s.setMaxIteration2(ui_->maxIteration2SpinBox->value());
    s.setMaxIteration3(ui_->maxIteration3SpinBox->value());
    s.smooth();

    emit processed();
}
