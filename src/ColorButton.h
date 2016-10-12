// =================================================================== //
// Copyright (C) 2016 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef COLOR_BUTTON_H
#define COLOR_BUTTON_H

#include <QtWidgets/QPushButton>

/*!
 * \class   ColorButton
 * \brief   The ColorButton class provides the push button with a color dialog window.
 */
class ColorButton : public QPushButton
{
    Q_OBJECT

public:
    explicit ColorButton(QWidget* parent);
    virtual ~ColorButton();

    QColor getColor() const;

signals:
    void colorChanged(const QColor& color);

public slots:
    void setColor(const QColor& color);

protected:
    void mousePressEvent(QMouseEvent* event);
    void paintEvent(QPaintEvent* event);
    void nextCheckState();

    QColor  color_;
    bool    picked_;

private:
    Q_DISABLE_COPY(ColorButton)
};

#endif // COLOR_BUTTON_H
