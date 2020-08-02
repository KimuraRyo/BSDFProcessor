// =================================================================== //
// Copyright (C) 2020 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef OPEN_SSDD_DIALOG_H
#define OPEN_SSDD_DIALOG_H

#include "ui_OpenSsddDialog.h"

/*!
 * \class   OpenSsddDialog
 * \brief   The OpenSsddDialog class provides the dialog window of an SSDD material file reader.
 */
class OpenSsddDialog : public QDialog
{
    Q_OBJECT

public:
    explicit OpenSsddDialog(QWidget* parent);
    ~OpenSsddDialog();

    void addBrdfItem()                  { ui_->typeComboBox->addItem(BRDF_STR); }
    void addBtdfItem()                  { ui_->typeComboBox->addItem(BTDF_STR); }
    void addSpecularReflectanceItem()   { ui_->typeComboBox->addItem(SR_STR); }
    void addSpecularTransmittanceItem() { ui_->typeComboBox->addItem(ST_STR); }

    bool isBrdf()                   { return (ui_->typeComboBox->currentText() == BRDF_STR); }
    bool isBtdf()                   { return (ui_->typeComboBox->currentText() == BTDF_STR); }
    bool isSpecularReflectance()    { return (ui_->typeComboBox->currentText() == SR_STR); }
    bool isSpecularTransmittance()  { return (ui_->typeComboBox->currentText() == ST_STR); }

    bool hasMultipleItems() { return ui_->typeComboBox->count() != 1; }

private:
    Q_DISABLE_COPY(OpenSsddDialog)

    static constexpr char BRDF_STR[]    = "BRDF";
    static constexpr char BTDF_STR[]    = "BTDF";
    static constexpr char SR_STR[]      = "Specular reflectance";
    static constexpr char ST_STR[]      = "Specular transmittance";

    Ui::OpenSsddDialogBase* ui_;
};

#endif // OPEN_SSDD_DIALOG_H
