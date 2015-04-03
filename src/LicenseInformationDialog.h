// =================================================================== //
// Copyright (C) 2015 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef LICENSE_INFORMATION_DIALOG_H
#define LICENSE_INFORMATION_DIALOG_H

#include "ui_LicenseInformation.h"

/*!
 * \class   LicenseInformationDialog
 * \brief   The LicenseInformationDialog class provides the dialog window of license information.
 */
class LicenseInformationDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LicenseInformationDialog(QWidget* parent) : QDialog(parent),
                                                         ui_(new Ui::LicenseInformationDialogBase)
    {
        ui_->setupUi(this);
    }

    ~LicenseInformationDialog()
    {
        delete ui_;
    }

private:
    Q_DISABLE_COPY(LicenseInformationDialog)

    Ui::LicenseInformationDialogBase* ui_;
};

#endif // LICENSE_INFORMATION_DIALOG_H
