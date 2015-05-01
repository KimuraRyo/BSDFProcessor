// =================================================================== //
// Copyright (C) 2015 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef OPEN_LIGHTTOOLS_BSDF_DIALOG_H
#define OPEN_LIGHTTOOLS_BSDF_DIALOG_H

#include "ui_OpenLightToolsBsdfDialog.h"

/*!
 * \class   OpenLightToolsBsdfDialog
 * \brief   The OpenLightToolsBsdfDialog class provides the dialog window of a LightTools BSDF file reader.
 */
class OpenLightToolsBsdfDialog : public QDialog
{
    Q_OBJECT

public:
    explicit OpenLightToolsBsdfDialog(QWidget* parent);
    ~OpenLightToolsBsdfDialog();

    void addFrontBrdfItem() { ui_->typeComboBox->addItem("Front side BRDF"); }
    void addFrontBtdfItem() { ui_->typeComboBox->addItem("Front side BTDF"); }
    void addBackBrdfItem()  { ui_->typeComboBox->addItem("Back side BRDF"); }
    void addBackBtdfItem()  { ui_->typeComboBox->addItem("Back side BTDF"); }

    bool isFrontBrdf() { return (ui_->typeComboBox->currentText() == "Front side BRDF"); }
    bool isFrontBtdf() { return (ui_->typeComboBox->currentText() == "Front side BTDF"); }
    bool isBackBrdf()  { return (ui_->typeComboBox->currentText() == "Back side BRDF"); }
    bool isBackBtdf()  { return (ui_->typeComboBox->currentText() == "Back side BTDF"); }

    bool hasMultipleItems() { return ui_->typeComboBox->count() != 1; }

private:
    Q_DISABLE_COPY(OpenLightToolsBsdfDialog)

    Ui::OpenLightToolsBsdfDialogBase* ui_;
};

#endif // OPEN_LIGHTTOOLS_BSDF_DIALOG_H
