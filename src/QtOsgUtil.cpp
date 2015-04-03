// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "QtOsgUtil.h"

#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/ShapeDrawable>

QColor qt_osg_util::getBackgroundColor(QPushButton* button)
{
    QString buttonStyleSheet = button->styleSheet();
    int startIndex = buttonStyleSheet.indexOf("#");
    return QColor(buttonStyleSheet.mid(startIndex, 7));
}

void qt_osg_util::setBackgroundColor(QPushButton* button, const QColor& color)
{
    const QString bgColorStyle("QPushButton { background-color : %1; }");
    button->setStyleSheet(bgColorStyle.arg(color.name()));
}

QColor qt_osg_util::setBackgroundColor(QPushButton* button)
{
    QColor chosenColor = QColorDialog::getColor(getBackgroundColor(button));
    if (chosenColor.isValid()) {
        setBackgroundColor(button, chosenColor);
    }
    return chosenColor;
}
