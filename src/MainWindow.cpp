// =================================================================== //
// Copyright (C) 2014-2022 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "MainWindow.h"

#include <QtWidgets>

#include <osg/io_utils>
#include <osg/Version>
#include <osgDB/WriteFile>
#include <osgGA/TrackballManipulator>

#include <libbsdf/Brdf/Analyzer.h>
#include <libbsdf/Brdf/Processor.h>
#include <libbsdf/Brdf/TwoSidedMaterial.h>

#include <libbsdf/Common/SpectrumUtility.h>
#include <libbsdf/Common/Version.h>

#include <libbsdf/Reader/AstmReader.h>
#include <libbsdf/Reader/DdrReader.h>
#include <libbsdf/Reader/LightToolsBsdfReader.h>
#include <libbsdf/Reader/MerlBinaryReader.h>
#include <libbsdf/Reader/SdrReader.h>
#include <libbsdf/Reader/SsddReader.h>
#include <libbsdf/Reader/ZemaxBsdfReader.h>

#include <libbsdf/Writer/DdrWriter.h>
#include <libbsdf/Writer/SsddWriter.h>

#include "AboutDialog.h"
#include "DoubleValidator.h"
#include "OpenAstmDialog.h"
#include "OpenLightToolsBsdfDialog.h"
#include "OpenSsddDialog.h"
#include "Settings.h"
#include "SceneUtil.h"
#include "Version.h"

const double MAX_LIGHT_INTENSITY_SLIDER = 2.0;
const double MAX_ENVIRONMENT_INTENSITY_SLIDER = 2.0;

const QString readOnlyStyleSheet = "QLineEdit { background-color: rgba(255, 255, 255, 0); }";

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent),
                                          cosineCorrected_(false),
                                          signalEmittedFromUi_(true),
                                          ui_(new Ui::MainWindowBase)
{
    lb::Log::setNotificationLevel(lb::Log::Level::INFO_MSG);
    osg::setNotifyLevel(osg::NOTICE);

    ui_->setupUi(this);

    characteristicDockWidget_       = new CharacteristicDockWidget(ui_->centralWidget);
    displayDockWidget_              = new DisplayDockWidget(ui_->centralWidget);
    pickDockWidget_                 = new PickDockWidget(ui_->centralWidget);
    propertyDockWidget_             = new PropertyDockWidget(ui_->centralWidget);
    reflectanceModelDockWidget_     = new ReflectanceModelDockWidget(ui_->centralWidget);
    transmittanceModelDockWidget_   = new TransmittanceModelDockWidget(ui_->centralWidget);
    smoothDockWidget_               = new SmoothDockWidget(ui_->centralWidget);
    scatteredSampleSetDockWidget_   = new ScatteredSampleSetDockWidget(ui_->centralWidget);
    insertAngleDockWidget_          = new InsertIncomingAzimuthalAngleDockWidget(ui_->centralWidget);

    transmittanceModelDockWidget_->setWindowTitle("Transmittance model");

    osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::ReaderWriter::Options::BUILD_KDTREES);

    setCentralWidget(ui_->graphOpenGLWidget);
    ui_->graphOpenGLWidget->setFocus();

    addDockWidget(Qt::BottomDockWidgetArea, characteristicDockWidget_);
    characteristicDockWidget_->hide();

    addDockWidget(Qt::RightDockWidgetArea, ui_->controlDockWidget);

    addDockWidget(Qt::RightDockWidgetArea, displayDockWidget_);
    displayDockWidget_->hide();

    addDockWidget(Qt::RightDockWidgetArea, ui_->renderingDockWidget);

    addDockWidget(Qt::RightDockWidgetArea, pickDockWidget_);

    addDockWidget(Qt::BottomDockWidgetArea, propertyDockWidget_);
    propertyDockWidget_->hide();

    addDockWidget(Qt::BottomDockWidgetArea, ui_->tableDockWidget);
    ui_->tableDockWidget->hide();

    addDockWidget(Qt::LeftDockWidgetArea, ui_->editorDockWidget);
    ui_->editorDockWidget->hide();

    addDockWidget(Qt::LeftDockWidgetArea, reflectanceModelDockWidget_);
    reflectanceModelDockWidget_->hide();

    addDockWidget(Qt::LeftDockWidgetArea, transmittanceModelDockWidget_);
    transmittanceModelDockWidget_->hide();

    addDockWidget(Qt::BottomDockWidgetArea, scatteredSampleSetDockWidget_);
    scatteredSampleSetDockWidget_->hide();

    addDockWidget(Qt::LeftDockWidgetArea, smoothDockWidget_);
    smoothDockWidget_->hide();

    addDockWidget(Qt::LeftDockWidgetArea, insertAngleDockWidget_);
    insertAngleDockWidget_->hide();

    resizeDocks({ ui_->controlDockWidget,
                  ui_->renderingDockWidget,
                  pickDockWidget_ },
                { ui_->controlDockWidget->minimumHeight(),
                  ui_->renderingDockWidget->maximumHeight(),
                  pickDockWidget_->minimumHeight() },
                Qt::Vertical);

    resizeDocks({ ui_->controlDockWidget }, { 280 }, Qt::Horizontal);

    tabifyDockWidget(ui_->tableDockWidget, propertyDockWidget_);
    tabifyDockWidget(ui_->tableDockWidget, characteristicDockWidget_);

    ui_->incomingPolarAngleLineEdit->setValidator(new DoubleValidator(this));
    ui_->incomingAzimuthalAngleLineEdit->setValidator(new DoubleValidator(this));

    ui_->statusbar->hide();

    data_.reset(new MaterialData);
    graphScene_.reset(new GraphScene);
    renderingScene_.reset(new RenderingScene);

    readSettings();
    setupRecentFiles();

    // Initialize the 3D graph view.
    graphScene_->setMaterialData(data_.get());
    graphScene_->createAxisAndScale();
    ui_->graphOpenGLWidget->setGraphScene(graphScene_.get());

    // Disable the small feature culling.
    osg::CullSettings::CullingMode cm = getGraphView()->getCamera()->getCullingMode();
    getGraphView()->getCamera()->setCullingMode(cm & ~osgUtil::CullVisitor::SMALL_FEATURE_CULLING);

    displayDockWidget_->setGraphScene(graphScene_.get());
    displayDockWidget_->setMaterialData(data_.get());

    pickDockWidget_->setGraphScene(graphScene_.get());
    pickDockWidget_->setMaterialData(data_.get());

    ui_->tableGraphicsView->setMaterialData(data_.get());
    ui_->tableGraphicsView->setLogPlotAction(ui_->graphOpenGLWidget->getLogPlotAction());

    // Initialize the rendering view.
    ui_->renderingOpenGLWidget->setRenderingScene(renderingScene_.get());

    createActions();

    lbInfo << "libbsdf-" << lb::getVersion();
    lbInfo << "OpenSceneGraph-" << osgGetVersion();
    lbInfo << "Qt-" << qVersion();
    lbInfo << "Eigen-" << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION;
    lbInfo << "Eigen SIMD: " << Eigen::SimdInstructionSetsInUse();
}

MainWindow::~MainWindow()
{
    delete ui_;
}

void MainWindow::openFile(const QString& fileName)
{
    QElapsedTimer timer;
    timer.start();

    bool hadData = (data_->getBrdf() ||
                    data_->getBtdf() ||
                    data_->getSpecularReflectances() ||
                    data_->getSpecularTransmittances());

    bool loaded = false;

    lb::FileType fileType = lb::reader_utility::classifyFile(fileName.toLocal8Bit().data());
    switch (fileType) {
        case lb::ASTM_FILE:
            loaded = openAstm(fileName);
            break;
        case lb::INTEGRA_DDR_FILE:
            loaded = openDdrDdt(fileName, lb::BRDF_DATA);
            break;
        case lb::INTEGRA_DDT_FILE:
            loaded = openDdrDdt(fileName, lb::BTDF_DATA);
            break;
        case lb::INTEGRA_SDR_FILE:
            loaded = openSdrSdt(fileName, lb::SPECULAR_REFLECTANCE_DATA);
            break;
        case lb::INTEGRA_SDT_FILE:
            loaded = openSdrSdt(fileName, lb::SPECULAR_TRANSMITTANCE_DATA);
            break;
        case lb::LIGHTTOOLS_FILE:
            loaded = openLightToolsBsdf(fileName);
            break;
        case lb::MERL_BINARY_FILE:
            loaded = openMerlBinary(fileName);
            break;
        case lb::SSDD_FILE:
            loaded = openSsdd(fileName);
            break;
        case lb::ZEMAX_FILE:
            loaded = openZemaxBsdf(fileName);
            break;
        default:
            break;
    }

    if (!loaded) {
        QMessageBox::warning(this, qApp->applicationName(), "Failed to load \"" + fileName + "\"");
        lbWarn
            << "[MainWindow::openFile] Failed to load: " << fileName.toLocal8Bit().data()
            << " (" << timer.elapsed() * 0.001f << "(s)" << ")";
        return;
    }

    data_->setFileType(fileType);

    if (!hadData) {
        viewFront();
    }

    QFileInfo fileInfo(fileName);
    this->setWindowTitle(fileInfo.fileName() + " - BSDF Processor");

    Settings::addRecentFileName(fileName);
    Settings::restoreRecentFileNames(&recentFileActionList_);

    lbInfo
        << "[MainWindow::openFile] fileName: " << fileName.toLocal8Bit().data()
        << " (" << timer.elapsed() * 0.001f << "(s)" << ")";
}

bool MainWindow::setupBrdf(std::shared_ptr<lb::Brdf> brdf, lb::DataType dataType)
{
    if (dataType != lb::BRDF_DATA &&
        dataType != lb::BTDF_DATA) {
        lbError << "[MainWindow::setupBrdf] Invalid data type: " << dataType;
        return false;
    }

    if (!brdf->getSampleSet()->validate()) {
        lbError << "[MainWindow::setupBrdf] Invalid BRDF.";
        return false;
    }

    if (cosineCorrected_) {
        lb::divideByCosineOutTheta(brdf.get());
    }

    data_->clearData();
    if (dataType == lb::BRDF_DATA) {
        data_->setBrdf(brdf);
    }
    else if (dataType == lb::BTDF_DATA) {
        data_->setBtdf(std::shared_ptr<lb::Btdf>(new lb::Btdf(brdf)));
    }

    renderingScene_->setData(brdf.get(), data_->getReflectances(), dataType);

    initializeUi();
    updateUi();

    return true;
}

bool MainWindow::setupSpecularReflectances(std::shared_ptr<lb::SampleSet2D> ss2, lb::DataType dataType)
{
    if (dataType != lb::SPECULAR_REFLECTANCE_DATA &&
        dataType != lb::SPECULAR_TRANSMITTANCE_DATA) {
        lbError << "[MainWindow::setupSpecularReflectances] Invalid data type: " << dataType;
        return false;
    }

    if (!ss2->validate()) {
        lbError << "[MainWindow::setupSpecularReflectances] Invalid specular reflectance.";
        return false;
    }

    data_->clearData();
    if (dataType == lb::SPECULAR_REFLECTANCE_DATA) {
        data_->setSpecularReflectances(ss2);
    }
    else if (dataType == lb::SPECULAR_TRANSMITTANCE_DATA) {
        data_->setSpecularTransmittances(ss2);
    }
    else {
        lbError << "[MainWindow::openSdrSdt] Invalid data type: " << dataType;
        return false;
    }

    renderingScene_->setData(nullptr, ss2.get(), dataType);

    initializeUi();
    updateUi();

    return true;
}

void MainWindow::setupBrdf(std::shared_ptr<lb::Brdf> brdf)
{
    lb::DataType dataType = data_->getDataType();
    if (dataType == lb::UNKNOWN_DATA) {
        lbError << "[MainWindow::setupBrdf] Invalid data type.";
        return;
    }

    setupBrdf(brdf, dataType);
}

void MainWindow::updateBrdf()
{
    if (data_->getBrdf()) {
        data_->updateBrdf();
    }
    else if (data_->getBtdf()) {
        data_->updateBtdf();
    }
    else {
        return;
    }

    renderingScene_->setData(data_->getBrdfData(), data_->getReflectances(), data_->getDataType());

    initializeUi();
    updateUi();
}

void MainWindow::openBxdfUsingDialog()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open BRDF/BTDF File", QString(),
                                                    "Surface Scattering Files (*.ssdd *.ddr *.ddt *.sdr *.sdt *.bsdf *.astm *.binary);;"
                                                    "SSDD (*.ssdd);;"
                                                    "Integra DDR (*.ddr);;"
                                                    "Integra DDT (*.ddt);;"
                                                    "Integra SDR (*.sdr);;"
                                                    "Integra SDR (*.sdt);;"
                                                    "LightTools/Zemax (*.bsdf);;"
                                                    "ASTM E1392-96(2002) (*.astm);;"
                                                    "MERL binary (*.binary)");

    if (fileName.isEmpty()) return;

    openFile(fileName);
}

void MainWindow::openCcbxdfUsingDialog()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open CCBRDF/CCBTDF File", QString(),
                                                    "CCBRDF/CCBTDF files (*.ddr *.ddt *.bsdf *.astm);;"
                                                    "Integra DDR (*.ddr);;"
                                                    "Integra DDT (*.ddt);;"
                                                    "LightTools/Zemax (*.bsdf);;"
                                                    "ASTM E1392-96(2002) (*.astm)");

    if (fileName.isEmpty()) return;

    cosineCorrected_ = true;
    openFile(fileName);
    cosineCorrected_ = false;
}

void MainWindow::openRecentFile()
{
    QAction* action = qobject_cast<QAction*>(sender());
    if (action) {
        openFile(action->data().toString());
    }
}

void MainWindow::exportDataUsingDialog()
{
    if (data_->isEmpty()) return;

    QString ssddFilterSSDD("SSDD (*.ssdd)");

    QString ddrFilter;
    if (data_->getBrdf()) {
        ddrFilter = "Integra DDR (*.ddr)";
    }
    else if (data_->getBtdf()) {
        ddrFilter = "Integra DDT (*.ddt)";
    }

    QString filter = ssddFilterSSDD + ";;" + ddrFilter;

    QString fileName = QFileDialog::getSaveFileName(this, "Export Surface Scattering File", QString(), filter);
    if (fileName.isEmpty()) return;

    exportFile(fileName);
}

void MainWindow::viewFront()
{
    scene_util::fitCameraPosition(getGraphView()->getCamera(),
                                  osg::Vec3(0.0, 1.0, 0.0),
                                  osg::Vec3(0.0, 0.0, 1.0),
                                  graphScene_->getBsdfGroup());
    updateCameraPosition();
}

void MainWindow::viewBack()
{
    scene_util::fitCameraPosition(getGraphView()->getCamera(),
                                  osg::Vec3(0.0, -1.0, 0.0),
                                  osg::Vec3(0.0, 0.0, 1.0),
                                  graphScene_->getBsdfGroup());
    updateCameraPosition();
}

void MainWindow::viewRight()
{
    scene_util::fitCameraPosition(getGraphView()->getCamera(),
                                  osg::Vec3(-1.0, 0.0, 0.0),
                                  osg::Vec3(0.0, 0.0, 1.0),
                                  graphScene_->getBsdfGroup());
    updateCameraPosition();
}

void MainWindow::viewLeft()
{
    scene_util::fitCameraPosition(getGraphView()->getCamera(),
                                  osg::Vec3(1.0, 0.0, 0.0),
                                  osg::Vec3(0.0, 0.0, 1.0),
                                  graphScene_->getBsdfGroup());
    updateCameraPosition();
}

void MainWindow::viewTop()
{
    scene_util::fitCameraPosition(getGraphView()->getCamera(),
                                  osg::Vec3(0.0, 0.0, 1.0),
                                  osg::Vec3(0.0, -1.0, 0.0),
                                  graphScene_->getBsdfGroup());
    updateCameraPosition();
}

void MainWindow::viewBottom()
{
    scene_util::fitCameraPosition(getGraphView()->getCamera(),
                                  osg::Vec3(0.0, 0.0, -1.0),
                                  osg::Vec3(0.0, 1.0, 0.0),
                                  graphScene_->getBsdfGroup());
    updateCameraPosition();
}

void MainWindow::about()
{
    AboutDialog dialog(this);
    dialog.exec();
}

void MainWindow::updateViews()
{
    graphScene_->updateInOutDirLine();

    ui_->graphOpenGLWidget->updateView();
    ui_->renderingOpenGLWidget->updateView();

    pickDockWidget_->displayReflectance();
}

void MainWindow::updateDisplayMode(QString modeName)
{
    initializeWavelengthUi(ui_->wavelengthSlider->value());
    initializeDisplayModeUi(modeName);

    if (modeName == getDisplayModeName(GraphScene::SAMPLE_POINTS_DISPLAY) ||
        modeName == getDisplayModeName(GraphScene::SAMPLE_POINT_LABELS_DISPLAY)) {
        int inThIndex = ui_->incomingPolarAngleSlider->value();
        int inPhIndex = ui_->incomingAzimuthalAngleSlider->value();

        graphScene_->updateGraphGeometry(inThIndex, inPhIndex, ui_->wavelengthSlider->value());

        float inPolarDegree = lb::toDegree(data_->getIncomingPolarAngle(inThIndex));
        ui_->incomingPolarAngleLineEdit->setText(QString::number(inPolarDegree));
        ui_->incomingPolarAngleLineEdit->home(false);

        float inAzimuthalDegree = lb::toDegree(data_->getIncomingAzimuthalAngle(inPhIndex));
        ui_->incomingAzimuthalAngleLineEdit->setText(QString::number(inAzimuthalDegree));
        ui_->incomingAzimuthalAngleLineEdit->home(false);
    }
    else {
        graphScene_->updateGraphGeometry(graphScene_->getInTheta(),
                                         graphScene_->getInPhi(),
                                         ui_->wavelengthSlider->value());
    }
    ui_->graphOpenGLWidget->updateView();

    pickDockWidget_->displayReflectance();
    createTable();
}

void MainWindow::updateIncomingPolarAngle(int index)
{
    if (!signalEmittedFromUi_) return;

    graphScene_->updateGraphGeometry(index,
                                     ui_->incomingAzimuthalAngleSlider->value(),
                                     ui_->wavelengthSlider->value());
    ui_->graphOpenGLWidget->update();

    float inPolarDegree = lb::toDegree(data_->getIncomingPolarAngle(index));
    ui_->incomingPolarAngleLineEdit->setText(QString::number(inPolarDegree));
    ui_->incomingPolarAngleLineEdit->home(false);

    pickDockWidget_->clearPickedValue();
    pickDockWidget_->displayReflectance();

    scatteredSampleSetDockWidget_->setInDirIndex(index);
}

void MainWindow::updateIncomingPolarAngle()
{
    float inTheta;
    if (!updateIncomingPolarAngle(&inTheta)) return;

    if (inTheta == graphScene_->getInTheta()) return;

    graphScene_->updateGraphGeometry(inTheta, graphScene_->getInPhi(), ui_->wavelengthSlider->value());
    ui_->graphOpenGLWidget->update();

    pickDockWidget_->clearPickedValue();
    pickDockWidget_->displayReflectance();
}

void MainWindow::updateIncomingAzimuthalAngle(int index)
{
    if (!signalEmittedFromUi_) return;

    graphScene_->updateGraphGeometry(ui_->incomingPolarAngleSlider->value(),
                                     index,
                                     ui_->wavelengthSlider->value());
    graphScene_->updateScaleInPlaneOfIncidence();
    ui_->graphOpenGLWidget->update();

    float inAzimuthalDegree = lb::toDegree(data_->getIncomingAzimuthalAngle(index));
    ui_->incomingAzimuthalAngleLineEdit->setText(QString::number(inAzimuthalDegree));
    ui_->incomingAzimuthalAngleLineEdit->home(false);

    pickDockWidget_->clearPickedValue();
    pickDockWidget_->displayReflectance();
}

void MainWindow::updateIncomingAzimuthalAngle()
{
    float inPhi;
    if (!updateIncomingAzimuthalAngle(&inPhi)) return;

    if (inPhi == graphScene_->getInPhi()) return;

    graphScene_->updateGraphGeometry(graphScene_->getInTheta(), inPhi, ui_->wavelengthSlider->value());
    graphScene_->updateScaleInPlaneOfIncidence();
    ui_->graphOpenGLWidget->update();

    pickDockWidget_->clearPickedValue();
    pickDockWidget_->displayReflectance();
}

void MainWindow::updateWavelength(int index)
{
    initializeWavelengthUi(index);

    GraphScene::DisplayMode dm = graphScene_->getDisplayMode();
    if (dm == GraphScene::SAMPLE_POINTS_DISPLAY ||
        dm == GraphScene::SAMPLE_POINT_LABELS_DISPLAY) {
        graphScene_->updateGraphGeometry(ui_->incomingPolarAngleSlider->value(),
                                         ui_->incomingAzimuthalAngleSlider->value(),
                                         index);
    }
    else {
        graphScene_->updateGraphGeometry(graphScene_->getInTheta(),
                                         graphScene_->getInPhi(),
                                         index);
    }
    ui_->graphOpenGLWidget->update();

    pickDockWidget_->updatePickedValue();
    pickDockWidget_->displayReflectance();
    createTable();
}

void MainWindow::updateInOutDir(const lb::Vec3& inDir, const lb::Vec3& outDir, bool graphUpdateRequested)
{
    graphScene_->updateInOutDirLine(inDir, outDir);
    ui_->graphOpenGLWidget->update();

    pickDockWidget_->updatePickedValue(inDir, outDir);

    if (graphUpdateRequested) {
        updateInDir(inDir);
    }
}

void MainWindow::updateInDir(const lb::Vec3& inDir)
{
    if (inDir.isZero()) return;

    GraphScene::DisplayMode dm = graphScene_->getDisplayMode();
    if (dm == GraphScene::ALL_INCOMING_POLAR_ANGLES_DISPLAY ||
        dm == GraphScene::ALL_INCOMING_AZIMUTHAL_ANGLES_DISPLAY ||
        dm == GraphScene::SAMPLE_POINTS_DISPLAY ||
        dm == GraphScene::SAMPLE_POINT_LABELS_DISPLAY) {
        QString modeName = getDisplayModeName(GraphScene::NORMAL_DISPLAY);
        ui_->graphModeComboBox->setCurrentText(modeName);
        initializeDisplayModeUi(modeName);
    }

    float inTheta, inPhi;
    lb::SphericalCoordinateSystem::fromXyz(inDir, &inTheta, &inPhi);

    QString inThetaStr = QString::number(lb::toDegree(inTheta), 'f', 3);
    ui_->incomingPolarAngleLineEdit->setText(inThetaStr);
    ui_->incomingPolarAngleLineEdit->home(false);

    QString inPhiStr = QString::number(lb::toDegree(inPhi), 'f', 3);
    ui_->incomingAzimuthalAngleLineEdit->setText(inPhiStr);
    ui_->incomingAzimuthalAngleLineEdit->home(false);

    graphScene_->updateGraphGeometry(inTheta, inPhi, ui_->wavelengthSlider->value());
    graphScene_->updateScaleInPlaneOfIncidence();
    ui_->graphOpenGLWidget->updateView();

    adjustIncomingPolarAngleSlider(inTheta);
    adjustIncomingAzimuthalAngleSlider(inPhi);

    pickDockWidget_->displayReflectance();
}

void MainWindow::updateLightPolarAngle(int angle)
{
    if (!signalEmittedFromUi_) return;

    signalEmittedFromUi_ = false;
    ui_->lightPolarAngleSpinBox->setValue(angle);
    signalEmittedFromUi_ = true;
}

void MainWindow::updateLightPolarAngle(double angle)
{
    if (signalEmittedFromUi_) {
        signalEmittedFromUi_ = false;
        ui_->lightPolarAngleSlider->setValue(static_cast<int>(angle));
        signalEmittedFromUi_ = true;
    }

    double theta = lb::toRadian(-angle);
    double phi = lb::toRadian(ui_->lightAzimuthalAngleSpinBox->value() - 90.0);

    lb::Vec3 dir = lb::SphericalCoordinateSystem::toXyz(theta, phi);
    renderingScene_->setLightDir(scene_util::toOsg(dir));
    ui_->renderingOpenGLWidget->updateView();
}

void MainWindow::updateLightAzimuthalAngle(int angle)
{
    if (!signalEmittedFromUi_) return;

    signalEmittedFromUi_ = false;
    ui_->lightAzimuthalAngleSpinBox->setValue(angle);
    signalEmittedFromUi_ = true;
}

void MainWindow::updateLightAzimuthalAngle(double angle)
{
    if (signalEmittedFromUi_) {
        signalEmittedFromUi_ = false;
        ui_->lightAzimuthalAngleSlider->setValue(static_cast<int>(angle));
        signalEmittedFromUi_ = true;
    }

    double theta = lb::toRadian(-ui_->lightPolarAngleSpinBox->value());
    double phi = lb::toRadian(angle - 90.0);

    lb::Vec3 dir = lb::SphericalCoordinateSystem::toXyz(theta, phi);
    renderingScene_->setLightDir(scene_util::toOsg(dir));
    ui_->renderingOpenGLWidget->updateView();
}

void MainWindow::updateLightIntensity(int intensity)
{
    if (!signalEmittedFromUi_) return;

    double val = static_cast<double>(intensity)
               / ui_->lightIntensitySlider->maximum()
               * MAX_LIGHT_INTENSITY_SLIDER;

    signalEmittedFromUi_ = false;
    ui_->lightIntensitySpinBox->setValue(val);
    signalEmittedFromUi_ = true;
}

void MainWindow::updateLightIntensity(double intensity)
{
    if (signalEmittedFromUi_) {
        double sliderVal = intensity
                         * ui_->lightIntensitySlider->maximum()
                         / MAX_LIGHT_INTENSITY_SLIDER;

        signalEmittedFromUi_ = false;
        ui_->lightIntensitySlider->setValue(static_cast<int>(sliderVal));
        signalEmittedFromUi_ = true;
    }

    renderingScene_->setLightIntensity(intensity);
    ui_->renderingOpenGLWidget->updateView();
}

void MainWindow::updateEnvironmentIntensity(int intensity)
{
    if (!signalEmittedFromUi_) return;

    double val = static_cast<double>(intensity)
               / ui_->environmentIntensitySlider->maximum()
               * MAX_ENVIRONMENT_INTENSITY_SLIDER;

    signalEmittedFromUi_ = false;
    ui_->environmentIntensitySpinBox->setValue(val);
    signalEmittedFromUi_ = true;
}

void MainWindow::updateEnvironmentIntensity(double intensity)
{
    if (signalEmittedFromUi_) {
        double sliderVal = intensity
                         * ui_->environmentIntensitySlider->maximum()
                         / MAX_ENVIRONMENT_INTENSITY_SLIDER;

        signalEmittedFromUi_ = false;
        ui_->environmentIntensitySlider->setValue(static_cast<int>(sliderVal));
        signalEmittedFromUi_ = true;
    }

    renderingScene_->setEnvironmentIntensity(intensity);

    osg::Camera* camera = ui_->renderingOpenGLWidget->getViewer()->getCamera();
    camera->setClearColor(osg::Vec4(intensity, intensity, intensity, 1.0));

    ui_->renderingOpenGLWidget->updateView();
}

void MainWindow::updateGlossyIntensity(int intensity)
{
    if (!signalEmittedFromUi_) return;

    double val = static_cast<double>(intensity)
               / ui_->glossyIntensitySlider->maximum()
               * maxGlossyIntensity_;

    signalEmittedFromUi_ = false;
    ui_->glossyIntensitySpinBox->setValue(val);
    signalEmittedFromUi_ = true;
}

void MainWindow::updateGlossyIntensity(double intensity)
{
    if (maxGlossyIntensity_ < intensity) {
        maxGlossyIntensity_ = intensity;
    }

    if (signalEmittedFromUi_) {
        int sliderVal = intensity / maxGlossyIntensity_ * ui_->glossyIntensitySlider->maximum();

        signalEmittedFromUi_ = false;
        ui_->glossyIntensitySlider->setValue(sliderVal);
        signalEmittedFromUi_ = true;
    }

    editBrdf(intensity,
             ui_->glossyShininessSpinBox->value(),
             ui_->diffuseIntensitySpinBox->value());
}

void MainWindow::updateGlossyShininess(int shininess)
{
    if (!signalEmittedFromUi_) return;

    double val = static_cast<double>(shininess)
               / ui_->glossyShininessSlider->maximum()
               * maxGlossyShininess_;

    signalEmittedFromUi_ = false;
    ui_->glossyShininessSpinBox->setValue(val);
    signalEmittedFromUi_ = true;
}

void MainWindow::updateGlossyShininess(double shininess)
{
    if (maxGlossyShininess_ < shininess) {
        maxGlossyShininess_ = shininess;
    }

    if (signalEmittedFromUi_) {
        int sliderVal = shininess / maxGlossyShininess_ * ui_->glossyShininessSlider->maximum();

        signalEmittedFromUi_ = false;
        ui_->glossyShininessSlider->setValue(sliderVal);
        signalEmittedFromUi_ = true;
    }

    editBrdf(ui_->glossyIntensitySpinBox->value(),
             shininess,
             ui_->diffuseIntensitySpinBox->value());
}

void MainWindow::updateDiffuseIntensity(int intensity)
{
    if (!signalEmittedFromUi_) return;

    double val = static_cast<double>(intensity)
               / ui_->diffuseIntensitySlider->maximum()
               * maxDiffuseIntensity_;

    signalEmittedFromUi_ = false;
    ui_->diffuseIntensitySpinBox->setValue(val);
    signalEmittedFromUi_ = true;
}

void MainWindow::updateDiffuseIntensity(double intensity)
{
    if (maxDiffuseIntensity_ < intensity) {
        maxDiffuseIntensity_ = intensity;
    }

    if (signalEmittedFromUi_) {
        int sliderVal = intensity / maxDiffuseIntensity_ * ui_->diffuseIntensitySlider->maximum();

        signalEmittedFromUi_ = false;
        ui_->diffuseIntensitySlider->setValue(sliderVal);
        signalEmittedFromUi_ = true;
    }

    editBrdf(ui_->glossyIntensitySpinBox->value(),
             ui_->glossyShininessSpinBox->value(),
             intensity);
}

void MainWindow::clearFileType()
{
    data_->setFileType(lb::UNKNOWN_FILE);
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    Settings settings;
    settings.save(*this);

    QMainWindow::closeEvent(event);
}

void MainWindow::readSettings()
{
    Settings settings;
    settings.restore(this);
}

void MainWindow::setupRecentFiles()
{
    for (auto i = 0; i < MAX_RECENT_FILES; ++i) {
        QAction* action = new QAction(this);
        action->setVisible(false);
        QObject::connect(action, SIGNAL(triggered()), this, SLOT(openRecentFile()));
        recentFileActionList_.append(action);
        ui_->menuRecentFiles->addAction(recentFileActionList_.at(i));
    }

    Settings::restoreRecentFileNames(&recentFileActionList_);
}

void MainWindow::createActions()
{
    ui_->viewMenu->addAction(ui_->graphOpenGLWidget->getLogPlotAction());
    ui_->viewMenu->addSeparator();
    ui_->viewMenu->addAction(ui_->controlDockWidget->toggleViewAction());
    ui_->viewMenu->addAction(displayDockWidget_->toggleViewAction());
    ui_->viewMenu->addAction(ui_->renderingDockWidget->toggleViewAction());
    ui_->viewMenu->addAction(pickDockWidget_->toggleViewAction());
    ui_->viewMenu->addAction(ui_->tableDockWidget->toggleViewAction());
    ui_->viewMenu->addAction(propertyDockWidget_->toggleViewAction());
    ui_->viewMenu->addAction(characteristicDockWidget_->toggleViewAction());
#if defined(DEVELOPER)
    ui_->viewMenu->addAction(scatteredSampleSetDockWidget_->toggleViewAction());
#endif
    ui_->viewMenu->addSeparator();
    ui_->viewMenu->addAction(ui_->editorDockWidget->toggleViewAction());
    ui_->viewMenu->addAction(reflectanceModelDockWidget_->toggleViewAction());
    ui_->viewMenu->addAction(transmittanceModelDockWidget_->toggleViewAction());

    ui_->processorsMenu->addAction(smoothDockWidget_->toggleViewAction());
    ui_->processorsMenu->addAction(insertAngleDockWidget_->toggleViewAction());

    ui_->tableDockWidget->toggleViewAction()->setShortcut(Qt::CTRL + Qt::Key_T);

    connect(ui_->actionOpenBrdf,    SIGNAL(triggered()), this, SLOT(openBxdfUsingDialog()));
    connect(ui_->actionOpenCcbrdf,  SIGNAL(triggered()), this, SLOT(openCcbxdfUsingDialog()));
    connect(ui_->actionExport,      SIGNAL(triggered()), this, SLOT(exportDataUsingDialog()));
    connect(ui_->actionQuit,        SIGNAL(triggered()), this, SLOT(close()));

    connect(ui_->actionViewFront,   SIGNAL(triggered()), this, SLOT(viewFront()));
    connect(ui_->actionViewBack,    SIGNAL(triggered()), this, SLOT(viewBack()));
    connect(ui_->actionViewRight,   SIGNAL(triggered()), this, SLOT(viewRight()));
    connect(ui_->actionViewLeft,    SIGNAL(triggered()), this, SLOT(viewLeft()));
    connect(ui_->actionViewTop,     SIGNAL(triggered()), this, SLOT(viewTop()));
    connect(ui_->actionViewBottom,  SIGNAL(triggered()), this, SLOT(viewBottom()));

    connect(ui_->actionAbout, SIGNAL(triggered()), this, SLOT(about()));

    connect(ui_->graphOpenGLWidget, SIGNAL(fileDropped(QString)), this, SLOT(openFile(QString)));
    connect(ui_->graphOpenGLWidget, SIGNAL(viewFront()),          this, SLOT(viewFront()));
    connect(ui_->graphOpenGLWidget, SIGNAL(picked(osg::Vec3)),  pickDockWidget_, SLOT(updateOutDir(osg::Vec3)));
    connect(ui_->graphOpenGLWidget, SIGNAL(clearPickedValue()), pickDockWidget_, SLOT(clearPickedValue()));

    connect(ui_->graphOpenGLWidget, SIGNAL(logPlotToggled(bool)),
            displayDockWidget_,     SLOT(toggleLogPlotCheckBox(bool)));

    connect(ui_->renderingOpenGLWidget, SIGNAL(inOutDirPicked(lb::Vec3, lb::Vec3)),
            this,                       SLOT(updateInOutDir(lb::Vec3, lb::Vec3)));
    connect(ui_->renderingOpenGLWidget, SIGNAL(clearPickedValue()),
            pickDockWidget_,            SLOT(clearPickedValue()));

    connect(ui_->tableGraphicsView, SIGNAL(inOutDirPicked(lb::Vec3, lb::Vec3)),
            this,                   SLOT(updateInOutDir(lb::Vec3, lb::Vec3)));
    connect(ui_->tableGraphicsView, SIGNAL(clearPickedValue()),
            pickDockWidget_,        SLOT(clearPickedValue()));

    connect(reflectanceModelDockWidget_,    SIGNAL(generated(std::shared_ptr<lb::Brdf>, lb::DataType)),
            this,                           SLOT(setupBrdf(std::shared_ptr<lb::Brdf>, lb::DataType)));
    connect(reflectanceModelDockWidget_,    SIGNAL(generated()),
            this,                           SLOT(clearFileType()));

    connect(transmittanceModelDockWidget_,  SIGNAL(generated(std::shared_ptr<lb::Brdf>, lb::DataType)),
            this,                           SLOT(setupBrdf(std::shared_ptr<lb::Brdf>, lb::DataType)));
    connect(transmittanceModelDockWidget_,  SIGNAL(generated()),
            this,                           SLOT(clearFileType()));

    connect(smoothDockWidget_,  SIGNAL(processed()),
            this,               SLOT(updateBrdf()));

    connect(insertAngleDockWidget_, SIGNAL(processed(std::shared_ptr<lb::Brdf>)),
            this,                   SLOT(setupBrdf(std::shared_ptr<lb::Brdf>)));

    connect(data_.get(), SIGNAL(computed()), this, SLOT(updateViews()));

    connect(displayDockWidget_, SIGNAL(redrawGraphRequested()), ui_->graphOpenGLWidget, SLOT(update()));
    connect(displayDockWidget_, SIGNAL(redrawTableRequested()), this,                   SLOT(createTable()));

    connect(pickDockWidget_, SIGNAL(redrawGraphRequested()), ui_->graphOpenGLWidget, SLOT(update()));
}

void MainWindow::initializeUi()
{
    this->setWindowTitle(qApp->applicationName());

    QComboBox* comboBox = ui_->graphModeComboBox;

    comboBox->clear();
    comboBox->addItem(getDisplayModeName(GraphScene::PHOTOMETRY_DISPLAY));
    comboBox->addItem(getDisplayModeName(GraphScene::NORMAL_DISPLAY));
    comboBox->addItem(getDisplayModeName(GraphScene::ALL_INCOMING_POLAR_ANGLES_DISPLAY));
    comboBox->addItem(getDisplayModeName(GraphScene::ALL_INCOMING_AZIMUTHAL_ANGLES_DISPLAY));
    comboBox->addItem(getDisplayModeName(GraphScene::ALL_WAVELENGTHS_DISPLAY));
    comboBox->addItem(getDisplayModeName(GraphScene::SAMPLE_POINTS_DISPLAY));
    comboBox->addItem(getDisplayModeName(GraphScene::SAMPLE_POINT_LABELS_DISPLAY));

    bool photometryDisabled = (data_->getColorModel() != lb::RGB_MODEL &&
                               data_->getColorModel() != lb::SPECTRAL_MODEL) ||
                               data_->getNumWavelengths() == 1;

    if (photometryDisabled) {
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::PHOTOMETRY_DISPLAY)));
    }

    if (data_->getNumWavelengths() == 1) {
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::ALL_WAVELENGTHS_DISPLAY)));
    }

    if (data_->getNumInTheta() == 1) {
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::ALL_INCOMING_POLAR_ANGLES_DISPLAY)));
    }

    if (data_->getNumInPhi() == 1) {
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::ALL_INCOMING_AZIMUTHAL_ANGLES_DISPLAY)));
    }

    if (!data_->isInDirDependentCoordinateSystem()) {
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::SAMPLE_POINTS_DISPLAY)));
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::SAMPLE_POINT_LABELS_DISPLAY)));
    }

    if (data_->getSpecularReflectances() ||
        data_->getSpecularTransmittances()) {
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::ALL_INCOMING_POLAR_ANGLES_DISPLAY)));
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::ALL_INCOMING_AZIMUTHAL_ANGLES_DISPLAY)));
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::ALL_WAVELENGTHS_DISPLAY)));
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::SAMPLE_POINTS_DISPLAY)));
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::SAMPLE_POINT_LABELS_DISPLAY)));
    }

    if (!photometryDisabled) {
        initializeDisplayModeUi(getDisplayModeName(GraphScene::PHOTOMETRY_DISPLAY));
    }
    else {
        initializeDisplayModeUi(getDisplayModeName(GraphScene::NORMAL_DISPLAY));
    }

    ui_->incomingPolarAngleSlider->setMaximum(data_->getNumInTheta() - 1);
    signalEmittedFromUi_ = false;
    ui_->incomingPolarAngleSlider->setValue(0);
    signalEmittedFromUi_ = true;

    float inPolarDegree = lb::toDegree(data_->getIncomingPolarAngle(0));
    ui_->incomingPolarAngleLineEdit->setText(QString::number(inPolarDegree));

    ui_->incomingAzimuthalAngleSlider->setMaximum(data_->getNumInPhi() - 1);
    signalEmittedFromUi_ = false;
    ui_->incomingAzimuthalAngleSlider->setValue(0);
    signalEmittedFromUi_ = true;

    float inAzimuthalDegree = lb::toDegree(data_->getIncomingAzimuthalAngle(0));
    ui_->incomingAzimuthalAngleLineEdit->setText(QString::number(inAzimuthalDegree));

    ui_->wavelengthSlider->setMaximum(data_->getNumWavelengths() - 1);
    ui_->wavelengthSlider->setValue(0);
    initializeWavelengthUi(0);
    ui_->wavelengthLineEdit->clear();

    graphScene_->updateGraphGeometry(ui_->incomingPolarAngleSlider->value(),
                                     ui_->incomingAzimuthalAngleSlider->value(),
                                     ui_->wavelengthSlider->value());

    displayDockWidget_->updateScene();

    auto trackball = dynamic_cast<osgGA::TrackballManipulator*>(getGraphView()->getCameraManipulator());
    if (trackball) {
        osg::Vec3d eye, center, up;
        getGraphView()->getCamera()->getViewMatrixAsLookAt(eye, center, up);
        trackball->setHomePosition(eye, osg::Vec3d(0.0, 0.0, 0.0), up);
        trackball->home(0.0);
    }

    graphScene_->useOit(false);
    ui_->graphOpenGLWidget->updateView();

    if (renderingScene_->getBrdf()) {
        ui_->lightPolarAngleSlider->setEnabled(true);
        ui_->lightPolarAngleSpinBox->setEnabled(true);
        ui_->lightAzimuthalAngleSlider->setEnabled(true);
        ui_->lightAzimuthalAngleSpinBox->setEnabled(true);
        ui_->lightIntensitySlider->setEnabled(true);
        ui_->lightIntensitySpinBox->setEnabled(true);
        if (ui_->lightIntensitySpinBox->value() == 0.0) {
            ui_->lightIntensitySpinBox->setValue(1.0);
        }
    }
    else if (renderingScene_->getReflectance()) {
        ui_->lightPolarAngleSlider->setDisabled(true);
        ui_->lightPolarAngleSpinBox->setDisabled(true);
        ui_->lightAzimuthalAngleSlider->setDisabled(true);
        ui_->lightAzimuthalAngleSpinBox->setDisabled(true);
        ui_->lightIntensitySlider->setDisabled(true);
        ui_->lightIntensitySpinBox->setDisabled(true);
        if (ui_->environmentIntensitySpinBox->value() == 0.0) {
            ui_->environmentIntensitySpinBox->setValue(0.5);
        }
    }

    if (data_->getBrdf() || data_->getBtdf()) {
        ui_->editorDockWidget->setEnabled(true);
        smoothDockWidget_->setEnabled(true);
    }
    else {
        ui_->editorDockWidget->setDisabled(true);
        smoothDockWidget_->setDisabled(true);
    }

    if ((data_->getBrdf() || data_->getBtdf()) && data_->isInDirDependentCoordinateSystem()) {
        insertAngleDockWidget_->setEnabled(true);
    }
    else {
        insertAngleDockWidget_->setDisabled(true);
    }

    // Avoid to perform functions invoked by emitting signals.
    signalEmittedFromUi_ = false;
    ui_->glossyIntensitySlider->setValue(ui_->glossyIntensitySlider->maximum() / 2);
    ui_->glossyShininessSlider->setValue(ui_->glossyShininessSlider->maximum() / 2);
    ui_->diffuseIntensitySlider->setValue(ui_->diffuseIntensitySlider->maximum() / 2);
    ui_->glossyIntensitySpinBox->setValue(1.0);
    ui_->glossyShininessSpinBox->setValue(1.0);
    ui_->diffuseIntensitySpinBox->setValue(1.0);
    signalEmittedFromUi_ = true;

    maxGlossyIntensity_ = 2.0;
    maxGlossyShininess_ = 2.0;
    maxDiffuseIntensity_ = 2.0;

    ui_->renderingOpenGLWidget->updateView();

    lb::Brdf* brdf = data_->getBrdfData();
    if (brdf) {
        smoothDockWidget_->setBrdf(brdf);
        insertAngleDockWidget_->setBrdf(brdf);
    }

    updateInOutDir(lb::Vec3(0.0, 0.0, 1.0), lb::Vec3::Zero(), true);

    pickDockWidget_->clearPickedValue();
    pickDockWidget_->displayReflectance();
    createTable();
}

void MainWindow::updateUi()
{
    displayDockWidget_->updateUi();
    pickDockWidget_->displayReflectance();
    propertyDockWidget_->updateData(*data_);
    characteristicDockWidget_->updateData(*data_);
    ui_->tableGraphicsView->fitView(0.9);
}

void MainWindow::initializeDisplayModeUi(QString modeName)
{
    ui_->incomingPolarAngleLineEdit->setStyleSheet("");
    ui_->incomingAzimuthalAngleLineEdit->setStyleSheet("");

    float inThetaDegree = lb::toDegree(graphScene_->getInTheta());
    ui_->incomingPolarAngleLineEdit->setText(QString::number(inThetaDegree));
    ui_->incomingPolarAngleLineEdit->home(false);

    float inPhiDegree = lb::toDegree(graphScene_->getInPhi());
    ui_->incomingAzimuthalAngleLineEdit->setText(QString::number(inPhiDegree));
    ui_->incomingAzimuthalAngleLineEdit->home(false);

    if (data_->getNumInTheta() == 1) {
        ui_->incomingPolarAngleSlider->setDisabled(true);
        ui_->incomingPolarAngleLineEdit->setDisabled(true);
    }
    else {
        ui_->incomingPolarAngleSlider->setEnabled(true);
        ui_->incomingPolarAngleLineEdit->setEnabled(true);
    }
    ui_->incomingPolarAngleLineEdit->setReadOnly(false);

    if (data_->getNumInPhi() == 1) {
        ui_->incomingAzimuthalAngleSlider->setDisabled(true);
        ui_->incomingAzimuthalAngleLineEdit->setDisabled(true);
    }
    else {
        ui_->incomingAzimuthalAngleSlider->setEnabled(true);
        ui_->incomingAzimuthalAngleLineEdit->setEnabled(true);
    }
    ui_->incomingAzimuthalAngleLineEdit->setReadOnly(false);

    if (data_->getNumWavelengths() == 1) {
        ui_->wavelengthSlider->setDisabled(true);
    }
    else {
        ui_->wavelengthSlider->setEnabled(true);
    }
    ui_->wavelengthLineEdit->setEnabled(true);

    pickDockWidget_->enablePickedValueLineEdit(true);
    pickDockWidget_->enablePickedReflectanceLineEdit(true);

    // Stop continuous update of the graph view.
    timer_.stop();

    if (modeName == getDisplayModeName(GraphScene::PHOTOMETRY_DISPLAY)) {
        graphScene_->setDisplayMode(GraphScene::PHOTOMETRY_DISPLAY);

        ui_->wavelengthSlider->setDisabled(true);
        ui_->wavelengthLineEdit->clear();
        ui_->wavelengthLineEdit->setDisabled(true);

        pickDockWidget_->updatePickedValue();
    }
    else if (modeName == getDisplayModeName(GraphScene::NORMAL_DISPLAY)) {
        graphScene_->setDisplayMode(GraphScene::NORMAL_DISPLAY);

        pickDockWidget_->updatePickedValue();
    }
    else if (modeName == getDisplayModeName(GraphScene::ALL_INCOMING_POLAR_ANGLES_DISPLAY)) {
        graphScene_->setDisplayMode(GraphScene::ALL_INCOMING_POLAR_ANGLES_DISPLAY);

        ui_->incomingPolarAngleSlider->setDisabled(true);
        ui_->incomingPolarAngleLineEdit->clear();
        ui_->incomingPolarAngleLineEdit->setDisabled(true);

        pickDockWidget_->clearPickedValueLineEdit();
        pickDockWidget_->enablePickedValueLineEdit(false);

        pickDockWidget_->clearPickedReflectanceLineEdit();
        pickDockWidget_->enablePickedReflectanceLineEdit(false);
    }
    else if (modeName == getDisplayModeName(GraphScene::ALL_INCOMING_AZIMUTHAL_ANGLES_DISPLAY)) {
        graphScene_->setDisplayMode(GraphScene::ALL_INCOMING_AZIMUTHAL_ANGLES_DISPLAY);

        ui_->incomingAzimuthalAngleSlider->setDisabled(true);
        ui_->incomingAzimuthalAngleLineEdit->clear();
        ui_->incomingAzimuthalAngleLineEdit->setDisabled(true);

        pickDockWidget_->clearPickedValueLineEdit();
        pickDockWidget_->enablePickedValueLineEdit(false);

        pickDockWidget_->clearPickedReflectanceLineEdit();
        pickDockWidget_->enablePickedReflectanceLineEdit(false);
    }
    else if (modeName == getDisplayModeName(GraphScene::ALL_WAVELENGTHS_DISPLAY)) {
        graphScene_->setDisplayMode(GraphScene::ALL_WAVELENGTHS_DISPLAY);

        ui_->wavelengthSlider->setDisabled(true);
        ui_->wavelengthLineEdit->setDisabled(true);
        ui_->wavelengthLineEdit->clear();

        pickDockWidget_->clearPickedValueLineEdit();
        pickDockWidget_->enablePickedValueLineEdit(false);

        pickDockWidget_->clearPickedReflectanceLineEdit();
        pickDockWidget_->enablePickedReflectanceLineEdit(false);
    }
    else if (modeName == getDisplayModeName(GraphScene::SAMPLE_POINTS_DISPLAY)) {
        graphScene_->setDisplayMode(GraphScene::SAMPLE_POINTS_DISPLAY);

        ui_->incomingPolarAngleLineEdit->setReadOnly(true);
        ui_->incomingPolarAngleLineEdit->setStyleSheet(readOnlyStyleSheet);
        ui_->incomingAzimuthalAngleLineEdit->setReadOnly(true);
        ui_->incomingAzimuthalAngleLineEdit->setStyleSheet(readOnlyStyleSheet);

        pickDockWidget_->clearPickedValue();
    }
    else if (modeName == getDisplayModeName(GraphScene::SAMPLE_POINT_LABELS_DISPLAY)) {
        graphScene_->setDisplayMode(GraphScene::SAMPLE_POINT_LABELS_DISPLAY);

        ui_->incomingPolarAngleLineEdit->setReadOnly(true);
        ui_->incomingPolarAngleLineEdit->setStyleSheet(readOnlyStyleSheet);
        ui_->incomingAzimuthalAngleLineEdit->setReadOnly(true);
        ui_->incomingAzimuthalAngleLineEdit->setStyleSheet(readOnlyStyleSheet);

        pickDockWidget_->clearPickedValue();

        // Start continuous update of the graph view for osgText::FadeText.
        connect(&timer_, SIGNAL(timeout()), ui_->graphOpenGLWidget, SLOT(update()));
        timer_.start(100);
    }
    else {
        lbError << "[MainWindow::initializeDisplayModeUi] Unknown mode: " << modeName.toStdString();
        return;
    }
}

void MainWindow::initializeWavelengthUi(int index)
{
    if (data_->getColorModel() == lb::MONOCHROMATIC_MODEL &&
        data_->getWavelength(index) == 0.0f) {
        ui_->wavelengthLabel->setText("Channel:");
        ui_->wavelengthLineEdit->clear();
    }
    else if (data_->getColorModel() == lb::RGB_MODEL) {
        ui_->wavelengthLabel->setText("Channel:");
        switch (index) {
            case 0:  ui_->wavelengthLineEdit->setText("R"); break;
            case 1:  ui_->wavelengthLineEdit->setText("G"); break;
            case 2:  ui_->wavelengthLineEdit->setText("B"); break;
            default: assert(false);
        }
    }
    else if (data_->getColorModel() == lb::XYZ_MODEL) {
        ui_->wavelengthLabel->setText("Channel:");
        switch (index) {
            case 0:  ui_->wavelengthLineEdit->setText("X"); break;
            case 1:  ui_->wavelengthLineEdit->setText("Y"); break;
            case 2:  ui_->wavelengthLineEdit->setText("Z"); break;
            default: assert(false);
        }
    }
    else if (data_->getColorModel() == lb::SPECTRAL_MODEL) {
        float wavelength = data_->getWavelength(index);
        ui_->wavelengthLabel->setText("Wavelength:");
        ui_->wavelengthLineEdit->setText(QString::number(wavelength) + "nm");
    }
    else {
        lbError << "[MainWindow::initializeWavelengthUi] Unknown color model: " << data_->getColorModel();
    }
}

void MainWindow::adjustIncomingPolarAngleSlider(float inTheta)
{
    if (data_->isEmpty()) return;

    // Find the nearest index and adjust the slider.
    int nearestIndex = 0;
    lb::Arrayf inThetaArray = data_->getReflectances()->getThetaArray();
    float minDiff = lb::SphericalCoordinateSystem::MAX_ANGLE0;
    for (int i = 0; i < inThetaArray.size(); ++i) {
        float diff = std::abs(inTheta - inThetaArray[i]);
        if (minDiff > diff) {
            minDiff = diff;
            nearestIndex = i;
        }
    }

    signalEmittedFromUi_ = false;
    ui_->incomingPolarAngleSlider->setValue(nearestIndex);
    signalEmittedFromUi_ = true;
}

void MainWindow::adjustIncomingAzimuthalAngleSlider(float inPhi)
{
    if (data_->isEmpty()) return;

    // Find the nearest index and adjust the slider.
    int nearestIndex = 0;
    lb::Arrayf inPhiArray = data_->getReflectances()->getPhiArray();
    float minDiff = lb::SphericalCoordinateSystem::MAX_ANGLE1;
    for (int i = 0; i < inPhiArray.size(); ++i) {
        float diff = std::abs(inPhi - inPhiArray[i]);
        if (minDiff > diff) {
            minDiff = diff;
            nearestIndex = i;
        }
    }

    signalEmittedFromUi_ = false;
    ui_->incomingAzimuthalAngleSlider->setValue(nearestIndex);
    signalEmittedFromUi_ = true;
}

bool MainWindow::updateIncomingPolarAngle(float* inTheta)
{
    if (data_->isEmpty()) return false;

    GraphScene::DisplayMode dm = graphScene_->getDisplayMode();
    if (dm == GraphScene::SAMPLE_POINTS_DISPLAY ||
        dm == GraphScene::SAMPLE_POINT_LABELS_DISPLAY) {
        return false;
    }

    QString inThetaStr = ui_->incomingPolarAngleLineEdit->text();

    bool ok;
    float inThetaDegree = inThetaStr.toFloat(&ok);
    if (!ok) {
        inThetaDegree = lb::toDegree(graphScene_->getInTheta());
        ui_->incomingPolarAngleLineEdit->setText(QString::number(inThetaDegree));
        ui_->incomingPolarAngleLineEdit->home(false);
        return false;
    }

    if (inThetaDegree < 0.0f || inThetaDegree > 90.0f) {
        inThetaDegree = lb::clamp(inThetaDegree, 0.0f, 90.0f);
        ui_->incomingPolarAngleLineEdit->setText(QString::number(inThetaDegree));
        ui_->incomingPolarAngleLineEdit->home(false);
    }

    *inTheta = std::min(lb::toRadian(inThetaDegree), lb::SphericalCoordinateSystem::MAX_ANGLE0);

    adjustIncomingPolarAngleSlider(*inTheta);

    return true;
}

bool MainWindow::updateIncomingAzimuthalAngle(float* inPhi)
{
    if (data_->isEmpty()) return false;

    GraphScene::DisplayMode dm = graphScene_->getDisplayMode();
    if (dm == GraphScene::SAMPLE_POINTS_DISPLAY ||
        dm == GraphScene::SAMPLE_POINT_LABELS_DISPLAY) {
        return false;
    }

    QString inPhiStr = ui_->incomingAzimuthalAngleLineEdit->text();

    bool ok;
    float inPhiDegree = inPhiStr.toFloat(&ok);
    if (!ok) {
        inPhiDegree = lb::toDegree(graphScene_->getInPhi());
        ui_->incomingAzimuthalAngleLineEdit->setText(QString::number(inPhiDegree));
        ui_->incomingAzimuthalAngleLineEdit->home(false);
        return false;
    }

    if (inPhiDegree < 0.0f || inPhiDegree > 360.0f) {
        inPhiDegree = lb::clamp(inPhiDegree, 0.0f, 360.0f);
        ui_->incomingAzimuthalAngleLineEdit->setText(QString::number(inPhiDegree));
        ui_->incomingAzimuthalAngleLineEdit->home(false);
    }

    *inPhi = std::min(lb::toRadian(inPhiDegree), lb::SphericalCoordinateSystem::MAX_ANGLE1);

    adjustIncomingAzimuthalAngleSlider(*inPhi);

    return true;
}

QString MainWindow::getDisplayModeName(GraphScene::DisplayMode mode)
{
    switch (mode) {
        case GraphScene::PHOTOMETRY_DISPLAY: {
            return "Photometry";
            break;
        }
        case GraphScene::NORMAL_DISPLAY: {
            return "Normal";
            break;
        }
        case GraphScene::ALL_INCOMING_POLAR_ANGLES_DISPLAY: {
            return "All incoming polar angles";
            break;
        }
        case GraphScene::ALL_INCOMING_AZIMUTHAL_ANGLES_DISPLAY: {
            return "All incoming azimuthal angles";
            break;
        }
        case GraphScene::ALL_WAVELENGTHS_DISPLAY: {
            bool spectral = (data_->getColorModel() == lb::SPECTRAL_MODEL);
            return spectral ? "All wavelengths" : "All channels";
            break;
        }
        case GraphScene::SAMPLE_POINTS_DISPLAY: {
            return "Sample points";
            break;
        }
        case GraphScene::SAMPLE_POINT_LABELS_DISPLAY: {
            return "Sample point labels";
            break;
        }
        default: {
            return "";
        }
    }
}

bool MainWindow::openSsdd(const QString& fileName)
{
    lb::Material* material = lb::SsddReader::read(fileName.toLocal8Bit().data());
    if (!material) return false;

    OpenSsddDialog dialog(this);

    std::shared_ptr<lb::Bsdf> bsdf      = material->getBsdf();
    std::shared_ptr<lb::SampleSet2D> sr = material->getSpecularReflectances();
    std::shared_ptr<lb::SampleSet2D> st = material->getSpecularTransmittances();

    std::shared_ptr<lb::Brdf> brdf;
    std::shared_ptr<lb::Btdf> btdf;
    if (bsdf) {
        brdf = material->getBsdf()->getBrdf();
        btdf = material->getBsdf()->getBtdf();

        if (brdf) { dialog.addBrdfItem(); }
        if (btdf) { dialog.addBtdfItem(); }
    }

    if (sr) { dialog.addSpecularReflectanceItem(); }
    if (st) { dialog.addSpecularTransmittanceItem(); }

    if (dialog.hasMultipleItems()) {
        if (dialog.exec() == QDialog::Rejected) {
            delete material;
            return false;
        }
    }

    if (dialog.isBrdf()) {
        setupBrdf(brdf, lb::BRDF_DATA);
    }

    if (dialog.isBtdf()) {
        setupBrdf(btdf->getBrdf(), lb::BTDF_DATA);
    }

    if (dialog.isSpecularReflectance()) {
        setupSpecularReflectances(sr, lb::SPECULAR_REFLECTANCE_DATA);
    }

    if (dialog.isSpecularTransmittance()) {
        setupSpecularReflectances(st, lb::SPECULAR_TRANSMITTANCE_DATA);
    }

    delete material;

    return true;
}

bool MainWindow::openDdrDdt(const QString& fileName, lb::DataType dataType)
{
    lb::SpecularCoordinatesBrdf* brdf = lb::DdrReader::read(fileName.toLocal8Bit().data());
    if (!brdf) return false;

    return setupBrdf(std::shared_ptr<lb::Brdf>(brdf), dataType);
}

bool MainWindow::openSdrSdt(const QString& fileName, lb::DataType dataType)
{
    lb::SampleSet2D* ss2 = lb::SdrReader::read(fileName.toLocal8Bit().data());
    if (!ss2) return false;

    return setupSpecularReflectances(std::shared_ptr<lb::SampleSet2D>(ss2), dataType);
}

bool MainWindow::openLightToolsBsdf(const QString& fileName)
{
    lb::TwoSidedMaterial* material = lb::LightToolsBsdfReader::read(fileName.toLocal8Bit().data());
    if (!material) return false;

    OpenLightToolsBsdfDialog dialog(this);

    std::shared_ptr<lb::Brdf> frontBrdf = material->getFrontMaterial()->getBsdf()->getBrdf();
    std::shared_ptr<lb::Btdf> frontBtdf = material->getFrontMaterial()->getBsdf()->getBtdf();
    std::shared_ptr<lb::Brdf> backBrdf  = material->getBackMaterial()->getBsdf()->getBrdf();
    std::shared_ptr<lb::Btdf> backBtdf  = material->getBackMaterial()->getBsdf()->getBtdf();

    if (frontBrdf) { dialog.addFrontBrdfItem(); }
    if (frontBtdf) { dialog.addFrontBtdfItem(); }
    if (backBrdf)  { dialog.addBackBrdfItem(); }
    if (backBtdf)  { dialog.addBackBtdfItem(); }

    if (dialog.hasMultipleItems()) {
        if (dialog.exec() == QDialog::Rejected) {
            delete material;
            return false;
        }
    }

    lb::Brdf* brdf;
    lb::DataType dataType;
    if (dialog.isFrontBrdf()) {
        brdf = frontBrdf.get();
        dataType = lb::BRDF_DATA;
    }
    else if (dialog.isFrontBtdf()) {
        brdf = frontBtdf->getBrdf().get();
        dataType = lb::BTDF_DATA;
    }
    else if (dialog.isBackBrdf()) {
        brdf = backBrdf.get();
        dataType = lb::BRDF_DATA;
    }
    else if (dialog.isBackBtdf()) {
        brdf = backBtdf->getBrdf().get();
        dataType = lb::BTDF_DATA;
    }
    else {
        return false;
    }

    bool ok = setupBrdf(std::shared_ptr<lb::Brdf>(brdf->clone()), dataType);

    delete material;

    return ok;
}

bool MainWindow::openZemaxBsdf(const QString& fileName)
{
    lb::DataType dataType;
    lb::SpecularCoordinatesBrdf* brdf = lb::ZemaxBsdfReader::read(fileName.toLocal8Bit().data(), &dataType);
    if (!brdf) return false;

    return setupBrdf(std::shared_ptr<lb::Brdf>(brdf), dataType);
}

bool MainWindow::openAstm(const QString& fileName)
{
    OpenAstmDialog dialog(this);
    if (dialog.exec() == QDialog::Rejected) return false;

    lb::SphericalCoordinatesBrdf* brdf = lb::AstmReader::read(fileName.toLocal8Bit().data());
    if (!brdf) return false;

    return setupBrdf(std::shared_ptr<lb::Brdf>(brdf), dialog.getDataType());
}

bool MainWindow::openMerlBinary(const QString& fileName)
{
    lb::HalfDifferenceCoordinatesBrdf* brdf = lb::MerlBinaryReader::read(fileName.toLocal8Bit().data());
    if (!brdf) return false;

    return setupBrdf(std::shared_ptr<lb::Brdf>(brdf), lb::BRDF_DATA);
}

void MainWindow::exportFile(const QString& fileName)
{
    lb::FileType fileType;
    if (fileName.endsWith(".ssdd")) {
        fileType = lb::SSDD_FILE;
    }
    else if (fileName.endsWith(".ddr")) {
        fileType = lb::INTEGRA_DDR_FILE;
    }
    else if (fileName.endsWith(".ddt")) {
        fileType = lb::INTEGRA_DDT_FILE;
    }
    else {
        lbError << "[MainWindow::exportFile] Invalid file extension: " << fileName.toLocal8Bit().data();
        return;
    }

    bool saved;
    switch (fileType) {
        case lb::SSDD_FILE:
            saved = exportSsdd(fileName);
            break;
        case lb::INTEGRA_DDR_FILE:
            saved = exportDdrDdt(fileName, lb::BRDF_DATA);
            break;
        case lb::INTEGRA_DDT_FILE:
            saved = exportDdrDdt(fileName, lb::BTDF_DATA);
            break;
        default:
            saved = false;
            break;
    }

    if (!saved) {
        lbError << "[MainWindow::exportFile] fileName: " << fileName.toLocal8Bit().data();
        return;
    }

    Settings::addRecentFileName(fileName);
    Settings::restoreRecentFileNames(&recentFileActionList_);
}

std::shared_ptr<lb::Brdf> reduceAnglesUsingBilateralSymmetry(std::shared_ptr<lb::Brdf> brdf, float threshold)
{
    const lb::SampleSet* ss = brdf->getSampleSet();

    lb::Spectrum diffSp = lb::computeBilateralSymmetry(*brdf);
    float y = lb::SpectrumUtility::spectrumToY(diffSp, ss->getColorModel(), ss->getWavelengths());
    if (y < threshold) {
        brdf.reset(lb::reduceAnglesUsingBilateralSymmetry(*brdf));
    }

    return brdf;
};

std::shared_ptr<lb::Brdf> reduceAnglesUsingReciprocity(std::shared_ptr<lb::Brdf> brdf, float threshold)
{
    lb::HalfDifferenceCoordinatesBrdf* hdBrdf = dynamic_cast<lb::HalfDifferenceCoordinatesBrdf*>(brdf.get());

    if (!hdBrdf) return brdf;

    const lb::SampleSet* ss = brdf->getSampleSet();

    lb::Spectrum diffSp = lb::computeReciprocity(*brdf);
    float y = lb::SpectrumUtility::spectrumToY(diffSp, ss->getColorModel(), ss->getWavelengths());
    if (y < threshold) {
        brdf.reset(lb::reduceAnglesUsingReciprocity(*hdBrdf));
    }

    return brdf;
};

bool MainWindow::exportSsdd(const QString& fileName)
{
    constexpr float threshold = 0.005f;

    std::shared_ptr<lb::Brdf> brdf = data_->getBrdf();
    if (brdf) {
        brdf = reduceAnglesUsingBilateralSymmetry(brdf, threshold);
        brdf = reduceAnglesUsingReciprocity(brdf, threshold);
    }

    std::shared_ptr<lb::Btdf> btdf = data_->getBtdf();
    if (btdf) {
        std::shared_ptr<lb::Brdf> btdfData;
        btdfData = reduceAnglesUsingBilateralSymmetry(btdf->getBrdf(), threshold);
        btdfData = reduceAnglesUsingReciprocity(btdfData, threshold);

        if (btdfData != btdf->getBrdf()) {
            btdf.reset(new lb::Btdf(btdfData));
        }
    }

    std::shared_ptr<lb::Bsdf> bsdf = std::make_shared<lb::Bsdf>(brdf, btdf);

    std::shared_ptr<lb::SampleSet2D> specR;
    if (data_->getSpecularReflectances()) {
        specR = std::make_shared<lb::SampleSet2D>(*data_->getSpecularReflectances());
    }

    std::shared_ptr<lb::SampleSet2D> specT;
    if (data_->getSpecularTransmittances()) {
        specT = std::make_shared<lb::SampleSet2D>(*data_->getSpecularTransmittances());
    }

    std::unique_ptr<lb::Material> mat(new lb::Material(bsdf, specR, specT));

    lb::SsddWriter::DataFormat format = lb::SsddWriter::DataFormat::ASCII_DATA;
    std::string comments("Software: BSDFProcessor-" + std::string(getVersion()));
    return lb::SsddWriter::write(fileName.toLocal8Bit().data(), *mat, format, comments);
}

bool MainWindow::exportDdrDdt(const QString& fileName, lb::DataType dataType)
{
    lb::Brdf* brdf;
    if (dataType == lb::BRDF_DATA && data_->getBrdf()) {
        brdf = data_->getBrdf().get();
    }
    else if (dataType == lb::BTDF_DATA && data_->getBtdf()) {
        brdf = data_->getBtdf()->getBrdf().get();
    }
    else {
        lbError << "[MainWindow::exportDdrDdt] Invalid data for export.";
        return false;
    }

    std::string comments("Software: BSDFProcessor-" + std::string(getVersion()));
    return lb::DdrWriter::write(fileName.toLocal8Bit().data(), *brdf, dataType, comments);
}

void MainWindow::updateCameraPosition()
{
    osgGA::CameraManipulator* cm = getGraphView()->getCameraManipulator();
    if (cm) {
        osg::Vec3d eye, center, up;
        getGraphView()->getCamera()->getViewMatrixAsLookAt(eye, center, up);
        cm->setHomePosition(eye, osg::Vec3d(0.0, 0.0, 0.0), up);
        cm->home(0.0);
    }

    ui_->graphOpenGLWidget->update();
}

void MainWindow::createTable()
{
    float gamma = displayDockWidget_->getGamma();
    bool photometric = (graphScene_->getDisplayMode() == GraphScene::PHOTOMETRY_DISPLAY);
    ui_->tableGraphicsView->createTable(ui_->wavelengthSlider->value(), gamma, photometric);
}

void MainWindow::editBrdf(lb::Spectrum::Scalar glossyIntensity,
                          lb::Spectrum::Scalar glossyShininess,
                          lb::Spectrum::Scalar diffuseIntensity)
{
    lbTrace << "[MainWindow::editBrdf]";

    lb::Brdf* brdf = data_->getBrdfData();
    if (!brdf) return;

    data_->editBrdf(glossyIntensity, glossyShininess, diffuseIntensity);

    GraphScene::DisplayMode dm = graphScene_->getDisplayMode();
    if (dm == GraphScene::SAMPLE_POINTS_DISPLAY ||
        dm == GraphScene::SAMPLE_POINT_LABELS_DISPLAY) {
        graphScene_->updateGraphGeometry(ui_->incomingPolarAngleSlider->value(),
                                         ui_->incomingAzimuthalAngleSlider->value(),
                                         ui_->wavelengthSlider->value());
    }
    else {
        graphScene_->updateGraphGeometry(graphScene_->getInTheta(),
                                         graphScene_->getInPhi(),
                                         ui_->wavelengthSlider->value());
    }

    graphScene_->updateInOutDirLine();

    ui_->graphOpenGLWidget->update();
    ui_->renderingOpenGLWidget->update();

    pickDockWidget_->updatePickedValue();
    pickDockWidget_->displayReflectance();
    propertyDockWidget_->updateData(*data_);
    characteristicDockWidget_->updateData(*data_);
    createTable();
}
