// =================================================================== //
// Copyright (C) 2016 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "ColorButton.h"

#include <QtGui/QPainter>
#include <QtWidgets/QColorDialog>

ColorButton::ColorButton(QWidget* parent)
                         : QPushButton(parent),
                           picked_(false)
{
#if defined(_WIN32)
    setFixedSize(70, 23);
#elif defined(__APPLE__)
    setFixedSize(70, 26);
#else
    setFixedSize(70, 26);
#endif
}

ColorButton::~ColorButton()
{
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

void ColorButton::mousePressEvent(QMouseEvent* event)
{
    QPushButton::mousePressEvent(event);

    picked_ = true;

    update();
}

void ColorButton::paintEvent(QPaintEvent* event)
{
    QPushButton::paintEvent(event);

    QPainter painter(this);

#if defined(_WIN32)
    const int leftMargin = 2;
#elif defined(__APPLE__)
    const int leftMargin = 5;
#else
    const int leftMargin = 0;
#endif

#if defined(_WIN32)
    const int topMargin = 2;
#elif defined(__APPLE__)
    const int topMargin = 1;
#else
    const int topMargin = 0;
#endif

    int sizeDecrease = 0;

    // Draw a frame.
#if defined(_WIN32)
    painter.setPen(Qt::NoPen);
    sizeDecrease++;
#else
    painter.setPen(QColor(180, 180, 180));
#endif
    if (picked_) {
        painter.setBrush(QColor(190, 190, 190));
    }
    else {
        painter.setBrush(QColor(240, 240, 240));
    }
    painter.drawRect(leftMargin,
                     topMargin,
                     width() - 2 - leftMargin,
                     height() - 2 - topMargin);

    // Draw a colored rectangle.
    if (picked_) {
        painter.setPen(QColor(90, 90, 90));
    }
    else {
        painter.setPen(QColor(140, 140, 140));
    }
    painter.setBrush(color_);

#if defined(_WIN32)
    const int frameWidth = 4;
#elif defined(__APPLE__)
    const int frameWidth = 6;
#else
    const int frameWidth = 6;
#endif
    painter.drawRect(frameWidth + leftMargin,
                     frameWidth + topMargin,
                     width() - frameWidth * 2 - 2 - leftMargin - sizeDecrease,
                     height() - frameWidth * 2 - 2 - topMargin - sizeDecrease);
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
