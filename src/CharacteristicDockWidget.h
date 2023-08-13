// =================================================================== //
// Copyright (C) 2020-2023 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef CHARACTERISTIC_DOCKWIDGET_H
#define CHARACTERISTIC_DOCKWIDGET_H

#include "ui_CharacteristicDockWidget.h"

#include "MaterialData.h"

/*!
 * \class   CharacteristicDockWidget
 * \brief   The CharacteristicDockWidget class provides the dock widget to show characteristics of data.
 */
class CharacteristicDockWidget : public QDockWidget
{
    Q_OBJECT

public:
    explicit CharacteristicDockWidget(QWidget* parent);
    ~CharacteristicDockWidget();

    void updateData(const MaterialData& materialData);

private:
    Q_DISABLE_COPY(CharacteristicDockWidget)

    void resizeEvent(QResizeEvent* event) override;

    void updateColumnDisplayMode();

    void addReflectanceItems();
    void add8DegreeReflectanceItems();
    void addBihemisphericalReflectanceItems(const lb::Brdf& brdf);
    void addReciprocityItems(const lb::Brdf& brdf);

    void addColors(QTreeWidgetItem*     parentItem,
                   const lb::Spectrum&  sp,
                   bool                 labUsed = true,
                   bool                 munsellUsed = true);

    void addSrgb(QTreeWidgetItem* parentItem, const lb::Vec3& xyz);
    void addAdobeRgb(QTreeWidgetItem* parentItem, const lb::Vec3& xyz);
    void addLab(QTreeWidgetItem* parentItem, const lb::Vec3& xyz);
    void addMunsell(QTreeWidgetItem* parentItem, const lb::Vec3& xyz);

    void setColorIcon(QTreeWidgetItem* item, const lb::Vec3& rgb);
    void setColorIcon(QTreeWidgetItem*      item,
                      const lb::Spectrum&   sp,
                      lb::ColorModel        cm,
                      const lb::Arrayf&     wavelengths);

    const MaterialData* data_;

    Ui::CharacteristicDockWidgetBase* ui_;
};

#endif // CHARACTERISTIC_DOCKWIDGET_H
