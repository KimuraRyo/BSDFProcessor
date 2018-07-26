// =================================================================== //
// Copyright (C) 2018 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef ANALYTIC_BSDF_DOCKWIDGET_H
#define ANALYTIC_BSDF_DOCKWIDGET_H

#include "ui_ReflectanceModelDockWidget.h"

#include <libbsdf/Brdf/Brdf.h>
#include <libbsdf/ReflectanceModel/ReflectanceModel.h>

/*!
 * \class   AnalyticBsdfDockWidget
 * \brief   The AnalyticBsdfDockWidget class provides the abstract dock widget for BRDF and BTDF generators.
 */
class AnalyticBsdfDockWidget : public QDockWidget
{
    Q_OBJECT

public:
    explicit AnalyticBsdfDockWidget(QWidget* parent);
    virtual ~AnalyticBsdfDockWidget();

signals:
    void generated(lb::Brdf* brdf, lb::DataType dataType);
    void generated();

protected slots:
    void updateParameterWidget(int index);
    void updateCoordSysWidget(int index);
    void updateParameter();

protected:
    Q_DISABLE_COPY(AnalyticBsdfDockWidget)

    virtual void initializeReflectanceModels() = 0;
    lb::Brdf* initializeBrdf(bool isotropic);

    std::map<std::string, lb::ReflectanceModel*> reflectanceModels_;

    /*! UI components and values for parameters of the current reflectance model. */
    std::map<QWidget*, lb::ReflectanceModel::Parameter*> currentParameters_;

    Ui::ReflectanceModelDockWidgetBase* ui_;
};

#endif // ANALYTIC_BSDF_DOCKWIDGET_H
