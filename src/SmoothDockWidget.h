// =================================================================== //
// Copyright (C) 2017 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef SMOOTH_DOCKWIDGET_H
#define SMOOTH_DOCKWIDGET_H

#include "ui_SmoothDockWidget.h"

#include <libbsdf/Brdf/Brdf.h>

/*!
 * \class   SmoothDockWidget
 * \brief   The SmoothDockWidget class provides the dock widget to smooth a BRDF.
 */
class SmoothDockWidget : public QDockWidget
{
    Q_OBJECT

public:
    explicit SmoothDockWidget(QWidget* parent);
    ~SmoothDockWidget();

    void setBrdf(lb::Brdf* brdf);

signals:
    void processed();

private slots:
    void process();

private:
    Q_DISABLE_COPY(SmoothDockWidget)

    lb::Brdf* brdf_;

    Ui::SmoothDockWidgetBase* ui_;
};

#endif // SMOOTH_DOCKWIDGET_H
