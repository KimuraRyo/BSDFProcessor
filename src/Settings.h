// =================================================================== //
// Copyright (C) 2020 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef SETTINGS_H
#define SETTINGS_H

#include "MainWindow.h"

/*!
 * \class   Settings
 * \brief   The Setting class provides functions to save and restore settings.
 */
class Settings
{
public:
    static void save(const MainWindow& mainWindow);
    static void restore(MainWindow* mainWindow);

    static void addRecentFileName(const QString& fileName);
    static void restoreRecentFileNames(QList<QAction*>* recentFileActionList);
};

#endif // SETTINGS_H
