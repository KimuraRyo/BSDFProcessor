// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef OPEN_ASTM_DIALOG_H
#define OPEN_ASTM_DIALOG_H

#include "ui_OpenAstmDialog.h"

#include <libbsdf/Common/Global.h>

/*!
 * \class   OpenAstmDialog
 * \brief   The OpenAstmDialog class provides the dialog window of an ASTM file reader.
 */
class OpenAstmDialog : public QDialog
{
    Q_OBJECT

public:
    explicit OpenAstmDialog(QWidget* parent);
    ~OpenAstmDialog();

    lb::DataType getDataType();

private:
    Q_DISABLE_COPY(OpenAstmDialog)

    Ui::OpenAstmDialogBase* ui_;
};

#endif // OPEN_ASTM_DIALOG_H
