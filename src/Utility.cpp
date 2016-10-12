// =================================================================== //
// Copyright (C) 2014-2016 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "Utility.h"

#include <QtWidgets/QColorDialog>

QColor util::getBackgroundColor(QPushButton* button)
{
    QString buttonStyleSheet = button->styleSheet();
    int startIndex = buttonStyleSheet.indexOf("#");
    return QColor(buttonStyleSheet.mid(startIndex, 7));
}

void util::setBackgroundColor(QPushButton* button, const QColor& color)
{
    const QString bgColorStyle("QPushButton { background-color : %1; }");
    button->setStyleSheet(bgColorStyle.arg(color.name()));
}

QColor util::setBackgroundColor(QPushButton* button)
{
    QColor chosenColor = QColorDialog::getColor(getBackgroundColor(button));
    if (chosenColor.isValid()) {
        setBackgroundColor(button, chosenColor);
    }
    return chosenColor;
}
