// =================================================================== //
// Copyright (C) 2022 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef SCATTERED_SAMPLE_SET_DOCKWIDGET_H
#define SCATTERED_SAMPLE_SET_DOCKWIDGET_H

#include "ui_ScatteredSampleSetDockWidget.h"

#include <libbsdf/Brdf/ScatteredSampleSet2D.h>

/*!
 * \class   ScatteredSampleSetDockWidget
 * \brief   The ScatteredSampleSetDockWidget class provides the dock widget for the 2D scattered samples.
 */
class ScatteredSampleSetDockWidget : public QDockWidget
{
    Q_OBJECT

public:
    explicit ScatteredSampleSetDockWidget(QWidget* parent);
    virtual ~ScatteredSampleSetDockWidget();

    void setScatteredSampleSet(std::vector<lb::ScatteredSampleSet2D> sss);

    void setInDirIndex(int index);

public slots:
    void fitView();

private:
    Q_DISABLE_COPY(ScatteredSampleSetDockWidget)

    std::vector<lb::ScatteredSampleSet2D> sss_;

    Ui::ScatteredSampleSetDockWidgetBase* ui_;
};

#endif // SCATTERED_SAMPLE_SET_DOCKWIDGET_H
