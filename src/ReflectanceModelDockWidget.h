// =================================================================== //
// Copyright (C) 2016 Kimura Ryo                                       //
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

private slots:
    void updateParameterWidget(int index);
    void updateCoordSysWidget(int index);
    void changeButtonColor();
    void updateParameter(double value);
    void generateBrdf();

private:
    Q_DISABLE_COPY(ReflectanceModelDockWidget)

    void initializeReflectanceModels();
    void initializeColorButton();
    lb::Brdf* initializeBrdf(bool isotropic);

    std::map<std::string, lb::ReflectanceModel*> reflectanceModels_;

    std::vector<QColor> colors_; /*!< Color coefficients for reflectance models. */

    /*! UI components and vlues for parameters of the current reflectance model. */
    std::map<QDoubleSpinBox*, float*> currentParameters_;

    Ui::ReflectanceModelDockWidgetBase* ui_;
    QPushButton* colorPushButton_;
};

#endif // REFLECTANCE_MODEL_DOCKWIDGET_H
