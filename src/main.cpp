// =================================================================== //
// Copyright (C) 2014-2020 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include <QtCore/QLocale>
#include <QtWidgets/QApplication>

#include "MainWindow.h"

int main(int argc, char** argv)
{
    QLocale::setDefault(QLocale::system());

    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
    QApplication::setHighDpiScaleFactorRoundingPolicy(Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);
#endif

    QApplication app(argc, argv);
    app.setOrganizationName("BSDFProcessorProject");
    app.setApplicationName("BSDF Processor");

    MainWindow* mainWindow = new MainWindow;
    mainWindow->show();

    const QStringList arguments = app.arguments();
    if (arguments.size() > 1 && QFile::exists(arguments.last())) {
        mainWindow->openFile(arguments.last());
    }

    return app.exec();
}
