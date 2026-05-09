// =================================================================== //
// Copyright (C) 2016-2026 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "ColorButton.h"

#include <QColorDialog>
#include <QPainter>

ColorButton::ColorButton(QWidget* parent) : QPushButton(parent), picked_(false) {}

ColorButton::~ColorButton() {}

QColor ColorButton::getColor() const
{
    return color_;
}

void ColorButton::setColor(const QColor& color)
{
    color_ = color;
    update();
}

void ColorButton::mousePressEvent(QMouseEvent* event)
{
    QPushButton::mousePressEvent(event);

    picked_ = true;
    update();
}

void ColorButton::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    if (picked_) {
        painter.setBrush(QColor(190, 190, 190));
    }
    else {
        painter.setBrush(QColor(240, 240, 240));
    }

    painter.fillRect(rect(), QColor(190, 190, 190));
    painter.fillRect(rect().adjusted(1, 1, -1, -1), painter.brush());

    constexpr int margin = 5;
    QRect         colorRect = rect().adjusted(margin, margin, -margin, -margin);

    painter.setPen(QColor(140, 140, 140, 40));
    painter.drawRect(colorRect.adjusted(-1, -1, 1, 1));
    painter.fillRect(colorRect, color_);
}

void ColorButton::nextCheckState()
{
    QPushButton::nextCheckState();

    picked_ = true;

    QColor origColor = color_;

    QColorDialog dialog(color_, this);
    connect(&dialog, SIGNAL(currentColorChanged(QColor)), SLOT(setColor(const QColor&)));

    if (dialog.exec() == QDialog::Accepted) {
        emit colorChanged(color_);
    }
    else{
        setColor(origColor);
    }

    picked_ = false;
}
