// =================================================================== //
// Copyright (C) 2018-2019 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef INCOMING_AZIMUTHAL_ANGLE_INSERTER_DOCKWIDGET_H
#define INCOMING_AZIMUTHAL_ANGLE_INSERTER_DOCKWIDGET_H

#include <memory>

#include "ui_InsertIncomingAzimuthalAngleDockWidget.h"

#include <libbsdf/Brdf/Brdf.h>

/*!
 * \class   InsertIncomingAzimuthalAngleDockWidget
 * \brief   The InsertIncomingAzimuthalAngleDockWidget class provides the dock widget to insert a BRDF.
 */
class InsertIncomingAzimuthalAngleDockWidget : public QDockWidget
{
    Q_OBJECT

public:
    explicit InsertIncomingAzimuthalAngleDockWidget(QWidget* parent);
    ~InsertIncomingAzimuthalAngleDockWidget();

    void setBrdf(lb::Brdf* brdf);

signals:
    void processed(std::shared_ptr<lb::Brdf> brdf);

private slots:
    void setFileName();
    void process();

private:
    Q_DISABLE_COPY(InsertIncomingAzimuthalAngleDockWidget)

    void openFile(const QString& fileName);

    lb::Brdf* baseBrdf_;
    lb::Brdf* insertedBrdf_;

    Ui::InsertIncomingAzimuthalAngleDockWidgetBase* ui_;
};

#endif // INCOMING_AZIMUTHAL_ANGLE_INSERTER_DOCKWIDGET_H
