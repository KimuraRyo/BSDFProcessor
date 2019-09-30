// =================================================================== //
// Copyright (C) 2016-2019 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef REFLECTANCE_MODEL_DOCKWIDGET_H
#define REFLECTANCE_MODEL_DOCKWIDGET_H

#include "AnalyticBsdfDockWidget.h"

/*!
 * \class   ReflectanceModelDockWidget
 * \brief   The ReflectanceModelDockWidget class provides the dock widget for BRDF
 *          generator with reflectance models.
 */
class ReflectanceModelDockWidget : public AnalyticBsdfDockWidget
{
    Q_OBJECT

public:
    explicit ReflectanceModelDockWidget(QWidget* parent);

signals:
    void generated(std::shared_ptr<lb::Brdf> brdf, lb::DataType dataType);
    void generated();

private slots:
    void generateBrdf();

private:
    Q_DISABLE_COPY(ReflectanceModelDockWidget)

    void initializeReflectanceModels();
};

#endif // REFLECTANCE_MODEL_DOCKWIDGET_H
