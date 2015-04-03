// =================================================================== //
// Copyright (C) 2015 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef ABOUT_DIALOG_H
#define ABOUT_DIALOG_H

#include "ui_AboutDialog.h"

/*!
 * \class   AboutDialog
 * \brief   The AboutDialog class provides the dialog window of software information.
 */
class AboutDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AboutDialog(QWidget* parent);
    ~AboutDialog();

private slots:
    void showLicenseInformation();

private:
    Q_DISABLE_COPY(AboutDialog)

    Ui::AboutDialogBase* ui_;
};

#endif // ABOUT_DIALOG_H
