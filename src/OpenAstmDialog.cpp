// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "OpenAstmDialog.h"

OpenAstmDialog::OpenAstmDialog(QWidget* parent) : QDialog(parent), ui_(new Ui::OpenAstmDialogBase)
{
    ui_->setupUi(this);
}

OpenAstmDialog::~OpenAstmDialog()
{
    delete ui_;
}

lb::DataType OpenAstmDialog::getDataType()
{
    if (ui_->typeComboBox->currentText() == "BRDF") {
        return lb::BRDF_DATA;
    }
    else if (ui_->typeComboBox->currentText() == "BTDF") {
        return lb::BTDF_DATA;
    }

    return lb::UNKNOWN_DATA;
}
