// =================================================================== //
// Copyright (C) 2018 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef TRANSMITTANCE_MODEL_DOCKWIDGET_H
#define TRANSMITTANCE_MODEL_DOCKWIDGET_H

#include "AnalyticBsdfDockWidget.h"

/*!
 * \class   TransmittanceModelDockWidget
 * \brief   The TransmittanceModelDockWidget class provides the dock widget for BTDF
 *          generator with transmittance models.
 */
class TransmittanceModelDockWidget : public AnalyticBsdfDockWidget
{
    Q_OBJECT

public:
    explicit TransmittanceModelDockWidget(QWidget* parent);

signals:
    void generated(lb::Brdf* brdf, lb::DataType dataType);
    void generated();

private slots:
    void generateBrdf();

private:
    Q_DISABLE_COPY(TransmittanceModelDockWidget)

    void initializeReflectanceModels();
};

#endif // TRANSMITTANCE_MODEL_DOCKWIDGET_H
