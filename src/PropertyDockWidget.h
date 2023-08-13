// =================================================================== //
// Copyright (C) 2020-2023 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef PROPERTY_DOCKWIDGET_H
#define PROPERTY_DOCKWIDGET_H

#include "ui_PropertyDockWidget.h"

#include "MaterialData.h"

/*!
 * \class   PropertyDockWidget
 * \brief   The PropertyDockWidget class provides the dock widget to show properties of data.
 */
class PropertyDockWidget : public QDockWidget
{
    Q_OBJECT

public:
    explicit PropertyDockWidget(QWidget* parent);
    ~PropertyDockWidget();

    void updateData(const MaterialData& materialData);

private:
    Q_DISABLE_COPY(PropertyDockWidget)

    void resizeEvent(QResizeEvent* event) override;

    void updateColumnDisplayMode();

    void addColorModelItems();
    void addDataTypeItems();
    void addParamTypeItems();
    void addParamTypeItems(const lb::Brdf& brdf);
    void addParamTypeItems(const lb::SampleSet2D& ss2);
    void addSourceTypeItems();
    void addWavelengthItems(QTreeWidgetItem* parentItem);

    const MaterialData* data_;

    Ui::PropertyDockWidgetBase* ui_;
};

#endif // PROPERTY_DOCKWIDGET_H
