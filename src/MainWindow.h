// =================================================================== //
// Copyright (C) 2014-2018 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include "ui_MainWindow.h"

#include "DisplayDockWidget.h"
#include "GraphScene.h"
#include "GraphWidget.h"
#include "InsertIncomingAzimuthalAngleDockWidget.h"
#include "MaterialData.h"
#include "ReflectanceModelDockWidget.h"
#include "TransmittanceModelDockWidget.h"
#include "SmoothDockWidget.h"
#include "RenderingScene.h"
#include "RenderingWidget.h"
#include "Utility.h"

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
    bool setupBrdf(lb::Brdf* brdf, lb::DataType dataType);
    void setupBrdf(lb::Brdf* brdf);
    void updateBrdf();

    void openBxdfUsingDialog();
    void openCcbxdfUsingDialog();

    void exportBxdfUsingDialog();

    void viewFront();
    void viewBack();
    void viewRight();
    void viewLeft();
    void viewTop();
    void viewBottom();

    void about();

    void updateViews();
    void requestRedrawGraph();

    void updateDisplayMode(QString modeName);
    void updateIncomingPolarAngle(int index);
    void updateIncomingPolarAngle();
    void updateIncomingAzimuthalAngle(int index);
    void updateIncomingAzimuthalAngle();
    void updateWavelength(int index);

    void updateInOutDirection(const lb::Vec3& inDir, const lb::Vec3& outDir);
    void updateInDirection(const lb::Vec3& inDir);
    void updateLightPolarAngle(int angle);
    void updateLightPolarAngle(double angle);
    void updateLightAzimuthalAngle(int angle);
    void updateLightAzimuthalAngle(double angle);
    void updateLightIntensity(int intensity);
    void updateLightIntensity(double intensity);
    void updateEnvironmentIntensity(int intensity);
    void updateEnvironmentIntensity(double intensity);

    void displayPickedValue(const osg::Vec3& position);
    void clearPickedValue();
    void displayReflectance();

    void updateGlossyIntensity(int intensity);
    void updateGlossyIntensity(double intensity);
    void updateGlossyShininess(int shininess);
    void updateGlossyShininess(double shininess);
    void updateDiffuseIntensity(int intensity);
    void updateDiffuseIntensity(double intensity);

    void createTable();

    void clearFileType();

private:
    Q_DISABLE_COPY(MainWindow)

    void createActions();

    void initializeUi();
    void initializeDisplayModeUi(QString modeName);
    void initializeWavelengthUi(int index);

    bool updateIncomingPolarAngleUi(float* inTheta);
    bool updateIncomingAzimuthalAngleUi(float* inPhi);

    QString getDisplayModeName(GraphScene::DisplayMode mode);

    bool openDdrDdt(const QString& fileName, lb::DataType dataType);
    bool openSdrSdt(const QString& fileName, lb::DataType dataType);
    bool openLightToolsBsdf(const QString& fileName);
    bool openZemaxBsdf(const QString& fileName);
    bool openAstm(const QString& fileName);
    bool openMerlBinary(const QString& fileName);

    void exportFile(const QString& fileName);
    void exportDdrDdt(const QString& fileName, lb::DataType dataType);

    osgViewer::View* getMainView() const { return ui_->mainViewerWidget->getView(0); }
    osgViewer::View* getRenderingView() const { return ui_->renderingViewerWidget->getView(0); }

    void updateCameraPosition();

    void editBrdf(lb::Spectrum::Scalar  glossyIntensity,
                  lb::Spectrum::Scalar  glossyShininess,
                  lb::Spectrum::Scalar  diffuseIntensity);

    MaterialData*   data_;
    GraphScene*     graphScene_;
    RenderingScene* renderingScene_;

    GraphWidget*        graphWidget_;
    RenderingWidget*    renderingWidget_;

    bool cosineCorrected_;

    double maxGlossyIntensity_;
    double maxGlossyShininess_;
    double maxDiffuseIntensity_;

    /*! This attribute holds whether a slot function is invoked by the signal from UI. */
    bool signalEmittedFromUi_;

    DisplayDockWidget*                      displayDockWidget_;
    ReflectanceModelDockWidget*             reflectanceModelDockWidget_;
    TransmittanceModelDockWidget*           transmittanceModelDockWidget_;
    SmoothDockWidget*                       smoothDockWidget_;
    InsertIncomingAzimuthalAngleDockWidget* insertAngleDockWidget_;

    Ui::MainWindowBase* ui_;
};

#endif // MAIN_WINDOW_H
