// =================================================================== //
// Copyright (C) 2014-2017 Kimura Ryo                                  //
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
#include "MaterialData.h"
#include "ReflectanceModelDockWidget.h"
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit MainWindow(QWidget* parent);
    ~MainWindow();

public slots:
    void openFile(const QString& fileName);

private slots:
    void setupBrdf(lb::Brdf* brdf, lb::DataType dataType = lb::BRDF_DATA);
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

    void updateDisplayMode(QString modeName);
    void updateIncomingPolarAngle(int index);
    void updateIncomingAzimuthalAngle(int index);
    void updateWavelength(int index);
    void useLogPlot(bool on);
    void updateBaseOfLogarithm(int index);

    void updateInOutDirection(const lb::Vec3& inDir, const lb::Vec3& outDir);
    void updateLightIntensity(double intensity);
    void updateEnvironmentIntensity(double intensity);

    void displayPickedValue(const osg::Vec3& position);
    void clearPickedValue();
    void displayReflectance();

    void updateGlossyIntensity(int intensity);
    void updateGlossyShininess(int shininess);
    void updateDiffuseIntensity(int intensity);

    void updateGlossyIntensity(double intensity);
    void updateGlossyShininess(double shininess);
    void updateDiffuseIntensity(double intensity);

    void clearFileType();

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

    void exportFile(const QString& fileName);
    void exportDdrDdt(const QString& fileName, lb::DataType dataType);

    osgViewer::View* getMainView() const { return ui_->mainViewerWidget->getView(0); }
    osgViewer::View* getRenderingView() const { return ui_->renderingViewerWidget->getView(0); }

    void updateCameraPosition();

    void createTable();

    void editBrdf(lb::Spectrum::Scalar  glossyIntensity,
                  lb::Spectrum::Scalar  glossyShininess,
                  lb::Spectrum::Scalar  diffuseIntensity);

    MaterialData*   data_;
    GraphScene*     graphScene_;
    RenderingScene* renderingScene_;

    GraphWidget*        graphWidget_;
    RenderingWidget*    renderingWidget_;

    bool cosineCorrected_;

    lb::Vec3 pickedInDir_, pickedOutDir_;

    double maxGlossyIntensity_;
    double maxGlossyShininess_;
    double maxDiffuseIntensity_;

    /*! This attribute holds whether a slot function is invoked by the signal from UI. */
    bool signalEmittedFromUi_;

    ReflectanceModelDockWidget* reflectanceModelDockWidget_;
    SmoothDockWidget*           smoothDockWidget_;

    Ui::MainWindowBase* ui_;
};

#endif // MAIN_WINDOW_H
