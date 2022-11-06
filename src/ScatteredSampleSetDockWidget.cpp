// =================================================================== //
// Copyright (C) 2022 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "ScatteredSampleSetDockWidget.h"

ScatteredSampleSetDockWidget::ScatteredSampleSetDockWidget(QWidget* parent)
    : QDockWidget(parent), ui_(new Ui::ScatteredSampleSetDockWidgetBase)
{
    ui_->setupUi(this);
}

ScatteredSampleSetDockWidget::~ScatteredSampleSetDockWidget()
{
    delete ui_;
}

void ScatteredSampleSetDockWidget::setScatteredSampleSet(std::vector<lb::ScatteredSampleSet2D> sss)
{
    sss_ = sss;
}

void ScatteredSampleSetDockWidget::setInDirIndex(int index)
{
    if (index >= sss_.size()) {
        ui_->scatteredSampleSetGraphicsView->initializeScene();
        return;
    }

    ui_->scatteredSampleSetGraphicsView->setScatteredSampleSet2D(&sss_.at(index));
}

void ScatteredSampleSetDockWidget::fitView()
{
    ui_->scatteredSampleSetGraphicsView->fitView();
}
