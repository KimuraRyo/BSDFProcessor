// =================================================================== //
// Copyright (C) 2015 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "OpenLightToolsBsdfDialog.h"

OpenLightToolsBsdfDialog::OpenLightToolsBsdfDialog(QWidget* parent) : QDialog(parent),
                                                                      ui_(new Ui::OpenLightToolsBsdfDialogBase)
{
    ui_->setupUi(this);
}

OpenLightToolsBsdfDialog::~OpenLightToolsBsdfDialog()
{
    delete ui_;
}
