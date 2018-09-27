// =================================================================== //
// Copyright (C) 2018 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef DOUBLE_VALIDATOR_H
#define DOUBLE_VALIDATOR_H

#include <QDoubleValidator>

/*!
 * \class   DoubleValidator
 * \brief   The DoubleValidator class provides the validator for text input.
 *
 * Some characters return QValidator::Acceptable instead of QValidator::Intermediate to be processed in functions.
 */
class DoubleValidator : public QDoubleValidator
{
public:
    explicit DoubleValidator(QObject* parent) : QDoubleValidator(parent) {}

    QValidator::State validate(QString& s, int& i) const
    {
        if (s.isEmpty() ||
            s == "-" ||
            s == "+" ||
            s == "." ||
            s == "e" ||
            s == "E") {
            // Invoke a slot.
            return QValidator::Acceptable;
        }

        return QDoubleValidator::validate(s, i);
    }
};

#endif // DOUBLE_VALIDATOR_H
