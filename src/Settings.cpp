// =================================================================== //
// Copyright (C) 2020 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "Settings.h"

#include <QtWidgets>

const QString MAIN_WINDOW_GEOMETRY      = "MainWindow/geometry";
const QString MAIN_WINDOW_STATE         = "MainWindow/windowState";
const QString MAIN_WINDOW_RECENT_FILES  = "MainWindow/recentFiles";

const QString GRAPH_WIDGET_LOG_PLOT     = "GraphWidget/logPlot";

const QString TABLE_WIDGET_BACK_SIDE    = "TableWidget/backSide";

void Settings::save(const MainWindow& mainWindow)
{
    QSettings settings;
    settings.setValue(MAIN_WINDOW_GEOMETRY, mainWindow.saveGeometry());
    settings.setValue(MAIN_WINDOW_STATE,    mainWindow.saveState());

    settings.setValue(GRAPH_WIDGET_LOG_PLOT, mainWindow.getGraphScene()->isLogPlot());

    settings.setValue(TABLE_WIDGET_BACK_SIDE, mainWindow.getTableView()->isBackSideShown());
}

void Settings::restore(MainWindow* mainWindow)
{
    QSettings settings;
    mainWindow->restoreGeometry(settings.value(MAIN_WINDOW_GEOMETRY).toByteArray());
    mainWindow->restoreState(settings.value(MAIN_WINDOW_STATE).toByteArray());

    bool logPlotUsed = settings.value(GRAPH_WIDGET_LOG_PLOT, false).toBool();
    mainWindow->getGraphScene()->useLogPlot(logPlotUsed);

    bool backSideShown = settings.value(TABLE_WIDGET_BACK_SIDE, false).toBool();
    mainWindow->getTableView()->showBackSide(backSideShown);
}

void Settings::addRecentFileName(const QString& fileName)
{
    QString fnStr = QDir::toNativeSeparators(fileName);

    QSettings settings;
    QStringList fileNames = settings.value(MAIN_WINDOW_RECENT_FILES).toStringList();
    fileNames.removeAll(fnStr);
    fileNames.prepend(fnStr);

    while (fileNames.size() > MainWindow::MAX_RECENT_FILES) {
        fileNames.removeLast();
    }

    settings.setValue(MAIN_WINDOW_RECENT_FILES, fileNames);
}

void Settings::restoreRecentFileNames(QList<QAction*>* recentFileActionList)
{
    QSettings settings;
    QStringList fileNames = settings.value(MAIN_WINDOW_RECENT_FILES).toStringList();

    int numNames = std::min(fileNames.size(), MainWindow::MAX_RECENT_FILES);

    for (int i = 0; i < numNames; ++i) {
        recentFileActionList->at(i)->setText(fileNames.at(i));
        recentFileActionList->at(i)->setData(fileNames.at(i));
        recentFileActionList->at(i)->setVisible(true);
    }

    for (int i = numNames; i < MainWindow::MAX_RECENT_FILES; ++i) {
        recentFileActionList->at(i)->setVisible(false);
    }
}
