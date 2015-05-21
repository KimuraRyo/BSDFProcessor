// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include "ui_MainWindow.h"

#include "GraphScene.h"
#include "GraphWidget.h"
#include "QtOsgUtil.h"
#include "RenderingScene.h"
#include "RenderingWidget.h"

/*!
 * \class   MainWindow
 * \brief   The MainWindow class provides the window frame.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent);
    ~MainWindow();

public slots:
    void openFile(const QString& fileName);

private slots:
    void openBxdfUsingDialog();

    void exportBxdfUsingDialog();
    void exportFile(const QString& fileName);

    void viewFront();
    void viewBack();
    void viewRight();
    void viewLeft();
    void viewTop();
    void viewBottom();

    void about();

    void updateDisplayMode(QString modeName);
    void updateIncomingPolarAngle(int index);
    void updateIncomingAzimuthalAngle(int index);
    void updateWavelength(int index);
    void useLogPlot(bool on);
    void updateBaseOfLogarithm(int index);

    void updateLightIntensity(double intensity);
    void updateEnvironmentIntensity(double intensity);

    void displayPickedValue(const osg::Vec3& position);
    void clearPickedValue();
    void displayReflectance();

private:
    Q_DISABLE_COPY(MainWindow)

    void createActions();

    void initializeUi();
    QString getDisplayModeName(GraphScene::DisplayMode mode);

    bool openDdrDdt(const QString& fileName, lb::DataType dataType);
    bool openSdrSdt(const QString& fileName, lb::DataType dataType);
    bool openLightToolsBsdf(const QString& fileName);
    bool openZemaxBsdf(const QString& fileName);
    bool openAstm(const QString& fileName);
    bool openMerlBinary(const QString& fileName);

    void exportDdrDdt(const QString& fileName, lb::DataType dataType);

    void setupBrdf(lb::Brdf* brdf, lb::DataType dataType = lb::BRDF_DATA);

    osgViewer::View* getMainView() const { return ui_->mainViewerWidget->getView(0); }
    osgViewer::View* getRenderingView() const { return ui_->renderingViewerWidget->getView(0); }

    void updateCameraPosition();

    void createTable();

    GraphScene*     graphScene_;
    RenderingScene* renderingScene_;

    GraphWidget*        graphWidget_;
    RenderingWidget*    renderingWidget_;

    Ui::MainWindowBase* ui_;
};

#endif // MAIN_WINDOW_H
