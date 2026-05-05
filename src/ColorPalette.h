// =================================================================== //
// Copyright (C) 2026 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

/*!
 * \file    ColorPalette.h
 * \brief   Color palette for UI.
 */

#ifndef COLOR_PALETTE_H
#define COLOR_PALETTE_H

#include <QtGui/QColor>

namespace ColorPalette {

namespace Default {
constexpr QColor Red(255, 56, 60);
constexpr QColor Orange(255, 141, 40);
constexpr QColor Yellow(255, 204, 0);
constexpr QColor Green(52, 199, 89);
constexpr QColor Mint(0, 200, 179);
constexpr QColor Teal(0, 195, 208);
constexpr QColor Cyan(0, 192, 232);
constexpr QColor Blue(0, 136, 255);
constexpr QColor Indigo(97, 85, 245);
constexpr QColor Purple(203, 48, 224);
constexpr QColor Pink(255, 45, 85);
constexpr QColor Brown(172, 127, 94);
constexpr QColor Gray(142, 142, 147);
} // namespace Default

namespace Light {
constexpr QColor Red(255, 97, 101);
constexpr QColor Orange(255, 160, 86);
constexpr QColor Yellow(254, 223, 67);
constexpr QColor Green(74, 217, 104);
constexpr QColor Mint(84, 223, 203);
constexpr QColor Teal(59, 221, 236);
constexpr QColor Cyan(109, 217, 255);
constexpr QColor Blue(92, 184, 255);
constexpr QColor Indigo(167, 170, 255);
constexpr QColor Purple(234, 141, 255);
constexpr QColor Pink(255, 138, 196);
constexpr QColor Brown(219, 166, 121);
constexpr QColor Gray(174, 174, 178);
} // namespace Light

} // namespace ColorPalette

#endif // COLOR_PALETTE_H