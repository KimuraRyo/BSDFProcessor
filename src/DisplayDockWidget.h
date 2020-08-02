// =================================================================== //
// Copyright (C) 2018-2019 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef DISPLAY_DOCKWIDGET_H
#define DISPLAY_DOCKWIDGET_H

#include "ui_DisplayDockWidget.h"

#include "GraphScene.h"
#include "MaterialData.h"

/*!
 * \class   DisplayDockWidget
 * \brief   The DisplayDockWidget class provides the dock widget to control lines and scales for 3D graph.
 */
class DisplayDockWidget : public QDockWidget
{
    Q_OBJECT

public:
    explicit DisplayDockWidget(QWidget* parent);
    ~DisplayDockWidget();

    void setGraphScene(GraphScene* scene);
    void setMaterialData(MaterialData* materialData) { data_ = materialData; }

    float getGamma();

    void updateScene();
    void updateUi();

signals:
    void redrawGraphRequested();
    void redrawTableRequested();

private slots:
    void useLogPlot(bool on);
    void updateBaseOfLogarithm(int index);

    void toggleLogPlotCheckBox(bool on);

    void setScaleLength1(double length);
    void setScaleLength2(double length);
    void showScaleInPlaneOfIncidence(bool on);
private:
    Q_DISABLE_COPY(DisplayDockWidget)

    GraphScene* graphScene_;
    MaterialData* data_;

    Ui::DisplayDockWidgetBase* ui_;
};

#endif // DISPLAY_DOCKWIDGET_H
