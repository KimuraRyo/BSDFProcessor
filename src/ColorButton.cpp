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

ColorButton::ColorButton(QWidget* parent) : QPushButton(parent), color_(Qt::black)
{
    connect(this, &QPushButton::clicked, this, &ColorButton::openColorDialog);
}

QColor ColorButton::getColor() const
{
    return color_;
}

void ColorButton::setColor(const QColor& color)
{
    color_ = color;
    update();
}

void ColorButton::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter painter(this);

    QColor buttonColor = palette().button().color();
    if (isDown()) {
        buttonColor = buttonColor.darker(120);
    }

    painter.fillRect(rect(), palette().mid().color());
    painter.fillRect(rect().adjusted(1, 1, -1, -1), buttonColor);

    constexpr int margin = 5;
    QRect         colorRect = rect().adjusted(margin, margin, -margin, -margin);

    if (!isEnabled()) {
        painter.setOpacity(0.4);
    }

    painter.setPen(QColor(140, 140, 140, 40));
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(colorRect.adjusted(-1, -1, 1, 1));
    painter.fillRect(colorRect, color_);
}

void ColorButton::openColorDialog()
{
    // Keep the pressed appearance while the modal dialog is open.
    setDown(true);

    QColor origColor = color_;

    QColorDialog dialog(color_, this);
    connect(&dialog, &QColorDialog::currentColorChanged, this, &ColorButton::setColor);

    if (dialog.exec() == QDialog::Accepted) {
        emit colorChanged(color_);
    }
    else {
        setColor(origColor);
    }

    setDown(false);
}
