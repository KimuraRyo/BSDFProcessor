// =================================================================== //
// Copyright (C) 2016-2017 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef REFLECTANCE_MODEL_DOCKWIDGET_H
#define REFLECTANCE_MODEL_DOCKWIDGET_H

#include "ui_ReflectanceModelDockWidget.h"

#include <libbsdf/Brdf/Brdf.h>
#include <libbsdf/ReflectanceModel/ReflectanceModel.h>

/*!
 * \class   ReflectanceModelDockWidget
 * \brief   The ReflectanceModelDockWidget class provides the dock widget for BRDF
*           generator with reflectance models.
 */
class ReflectanceModelDockWidget : public QDockWidget
{
    Q_OBJECT

public:
    explicit ReflectanceModelDockWidget(QWidget* parent);
    ~ReflectanceModelDockWidget();

signals:
    void generated(lb::Brdf* brdf, lb::DataType dataType);
    void generated();

private slots:
    void updateParameterWidget(int index);
    void updateCoordSysWidget(int index);
    void updateParameter();
    void generateBrdf();

private:
    Q_DISABLE_COPY(ReflectanceModelDockWidget)

    void initializeReflectanceModels();
    lb::Brdf* initializeBrdf(bool isotropic);

    std::map<std::string, lb::ReflectanceModel*> reflectanceModels_;

    /*! UI components and values for parameters of the current reflectance model. */
    std::map<QWidget*, lb::ReflectanceModel::Parameter*> currentParameters_;

    Ui::ReflectanceModelDockWidgetBase* ui_;
};

#endif // REFLECTANCE_MODEL_DOCKWIDGET_H
