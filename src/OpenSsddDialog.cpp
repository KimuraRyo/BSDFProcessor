// =================================================================== //
// Copyright (C) 2020 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "OpenSsddDialog.h"

OpenSsddDialog::OpenSsddDialog(QWidget* parent) : QDialog(parent),
                                                  ui_(new Ui::OpenSsddDialogBase)
{
    ui_->setupUi(this);
}

OpenSsddDialog::~OpenSsddDialog()
{
    delete ui_;
}
