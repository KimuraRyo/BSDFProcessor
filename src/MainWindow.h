// =================================================================== //
// Copyright (C) 2014-2022 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <memory>

#include <QTimer>

#include "ui_MainWindow.h"

#include "CharacteristicDockWidget.h"
#include "DisplayDockWidget.h"
#include "GraphScene.h"
#include "GraphWidget.h"
#include "PickDockWidget.h"
#include "InsertIncomingAzimuthalAngleDockWidget.h"
#include "MaterialData.h"
#include "PropertyDockWidget.h"
#include "ReflectanceModelDockWidget.h"
#include "RenderingScene.h"
#include "RenderingWidget.h"
#include "TransmittanceModelDockWidget.h"
#include "ScatteredSampleSetDockWidget.h"
#include "SmoothDockWidget.h"

/*!
 * \class   MainWindow
 * \brief   The MainWindow class provides the window frame.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    static constexpr int MAX_RECENT_FILES = 10;

    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

    const GraphScene* getGraphScene() const { return graphScene_.get(); }
    GraphScene*       getGraphScene()       { return graphScene_.get(); }

    const TableView* getTableView() const { return ui_->tableGraphicsView; }
    TableView*       getTableView()       { return ui_->tableGraphicsView; }

public slots:
    void openFile(const QString& fileName);

private slots:
    bool setupBrdf(std::shared_ptr<lb::Brdf> brdf, lb::DataType dataType);
    bool setupSpecularReflectances(std::shared_ptr<lb::SampleSet2D> ss2, lb::DataType dataType);
    void setupBrdf(std::shared_ptr<lb::Brdf> brdf);
    void updateBrdf();

    void openBxdfUsingDialog();
    void openCcbxdfUsingDialog();

    void openRecentFile();

    void exportDataUsingDialog();

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
    void updateIncomingPolarAngle();
    void updateIncomingAzimuthalAngle(int index);
    void updateIncomingAzimuthalAngle();
    void updateWavelength(int index);

    void updateInOutDir(const lb::Vec3& inDir, const lb::Vec3& outDir, bool graphUpdateRequested = true);
    void updateLightPolarAngle(int angle);
    void updateLightPolarAngle(double angle);
    void updateLightAzimuthalAngle(int angle);
    void updateLightAzimuthalAngle(double angle);
    void updateLightIntensity(int intensity);
    void updateLightIntensity(double intensity);
    void updateEnvironmentIntensity(int intensity);
    void updateEnvironmentIntensity(double intensity);

    void updateGlossyIntensity(int intensity);
    void updateGlossyIntensity(double intensity);
    void updateGlossyShininess(int shininess);
    void updateGlossyShininess(double shininess);
    void updateDiffuseIntensity(int intensity);
    void updateDiffuseIntensity(double intensity);

    // Raises a dockwidget to the top when it is shown.
    void raiseControlDockWidget();
    void raiseDisplayDockWidget();
    void raiseRenderingDockWidget();
    void raisePickDockWidget();
    void raiseTableDockWidget();
    void raisePropertyDockWidget();
    void raiseCharacteristicDockWidget();
    void raiseScatteredSampleSetDockWidget();
    void raiseEditorDockWidget();
    void raiseReflectanceModelDockWidget();
    void raiseTransmittanceModelDockWidget();
    void raiseSmoothDockWidget();
    void raiseInsertAngleDockWidget();

    void createTable();

    void clearFileType();

private:
    Q_DISABLE_COPY(MainWindow)

    void closeEvent(QCloseEvent* event) override;

    void readSettings();
    void setupRecentFiles();
    
    void createActions();

    void initializeUi();
    void updateUi();
    void initializeDisplayModeUi(QString modeName);
    void initializeWavelengthUi(int index);

    void adjustIncomingPolarAngleSlider(float inTheta);
    void adjustIncomingAzimuthalAngleSlider(float inPhi);

    bool updateIncomingPolarAngle(float* inTheta);
    bool updateIncomingAzimuthalAngle(float* inPhi);

    void updateInDir(const lb::Vec3& inDir);

    QString getDisplayModeName(GraphScene::DisplayMode mode);

    bool openSsdd(const QString& fileName);
    bool openDdrDdt(const QString& fileName, lb::DataType dataType);
    bool openSdrSdt(const QString& fileName, lb::DataType dataType);
    bool openLightToolsBsdf(const QString& fileName);
    bool openZemaxBsdf(const QString& fileName);
    bool openAstm(const QString& fileName);
    bool openMerlBinary(const QString& fileName);

    void exportFile(const QString& fileName);
    bool exportSsdd(const QString& fileName);
    bool exportDdrDdt(const QString& fileName, lb::DataType dataType);
    bool exportSdrSdt(const QString& fileName, lb::DataType dataType);

    osgViewer::View* getGraphView() const { return ui_->graphOpenGLWidget->getViewer(); }

    void updateCameraPosition();

    void editBrdf(lb::Spectrum::Scalar  glossyIntensity,
                  lb::Spectrum::Scalar  glossyShininess,
                  lb::Spectrum::Scalar  diffuseIntensity);

    std::unique_ptr<MaterialData>   data_;
    std::unique_ptr<GraphScene>     graphScene_;
    std::unique_ptr<RenderingScene> renderingScene_;

    bool cosineCorrected_;

    double maxGlossyIntensity_;
    double maxGlossyShininess_;
    double maxDiffuseIntensity_;

    /*! This attribute holds whether a slot function is invoked by the signal from UI. */
    bool signalEmittedFromUi_;

    CharacteristicDockWidget*               characteristicDockWidget_;
    DisplayDockWidget*                      displayDockWidget_;
    PickDockWidget*                         pickDockWidget_;
    PropertyDockWidget*                     propertyDockWidget_;
    ReflectanceModelDockWidget*             reflectanceModelDockWidget_;
    TransmittanceModelDockWidget*           transmittanceModelDockWidget_;
    ScatteredSampleSetDockWidget*           scatteredSampleSetDockWidget_;
    SmoothDockWidget*                       smoothDockWidget_;
    InsertIncomingAzimuthalAngleDockWidget* insertAngleDockWidget_;

    QList<QAction*> recentFileActionList_;

    QTimer timer_;

    Ui::MainWindowBase* ui_;
};

#endif // MAIN_WINDOW_H
