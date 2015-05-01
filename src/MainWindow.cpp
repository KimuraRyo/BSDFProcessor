// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "MainWindow.h"

#include <iostream>

#include <QtWidgets>

#include <osg/io_utils>
#include <osg/Version>
#include <osgDB/WriteFile>

#include <libbsdf/Brdf/Brdf.h>
#include <libbsdf/Brdf/Btdf.h>
#include <libbsdf/Brdf/HalfDifferenceCoordinatesBrdf.h>
#include <libbsdf/Brdf/SampleSet2D.h>
#include <libbsdf/Brdf/SpecularCoordinatesBrdf.h>
#include <libbsdf/Brdf/SphericalCoordinatesBrdf.h>
#include <libbsdf/Brdf/TwoSidedMaterial.h>

#include <libbsdf/Common/Version.h>

#include <libbsdf/Reader/AstmReader.h>
#include <libbsdf/Reader/DdrReader.h>
#include <libbsdf/Reader/LightToolsBsdfReader.h>
#include <libbsdf/Reader/MerlBinaryReader.h>
#include <libbsdf/Reader/ReaderUtility.h>
#include <libbsdf/Reader/SdrReader.h>
#include <libbsdf/Reader/ZemaxBsdfReader.h>

#include <libbsdf/Writer/DdrWriter.h>

#include "AboutDialog.h"
#include "OpenAstmDialog.h"
#include "OpenLightToolsBsdfDialog.h"
#include "SceneUtil.h"
#include "SpecularCenteredCoordinateSystem.h"

lb::Vec3 qcolorToVec3(const QColor& color)
{
    qreal r, g, b;
    color.getRgbF(&r, &g, &b);
    return lb::Vec3(r, g, b);
}

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui_(new Ui::MainWindowBase)
{
    ui_->setupUi(this);

    osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::ReaderWriter::Options::BUILD_KDTREES);

    setCentralWidget(ui_->mainViewerWidget);
    //ui_->mainViewerWidget->setFocus();

    addDockWidget(Qt::RightDockWidgetArea,  ui_->controlDockWidget);
    addDockWidget(Qt::RightDockWidgetArea,  ui_->renderingDockWidget);
    addDockWidget(Qt::RightDockWidgetArea,  ui_->informationDockWidget);
    addDockWidget(Qt::BottomDockWidgetArea, ui_->tableDockWidget);

    ui_->statusbar->hide();

    // Initialize a 3D graph view.
    {
        graphWidget_ = new GraphWidget(ViewerWidget::createQGLFormat());
        graphWidget_->resize(1024, 1024);
        ui_->mainViewerWidget->addViewWidget(graphWidget_);

        graphScene_ = new GraphScene;
        graphScene_->setCamera(getMainView()->getCamera());
        graphScene_->setCameraManipulator(getMainView()->getCameraManipulator());
        getMainView()->setSceneData(graphScene_->getRoot());
        graphWidget_->setGraphScene(graphScene_);
        ui_->tableGraphicsView->setGraphScene(graphScene_);
    }

    // Initialize a rendering view.
    {
        renderingWidget_ = new RenderingWidget(ViewerWidget::createQGLFormat());
        renderingWidget_->resize(1024, 1024);
        ui_->renderingViewerWidget->addViewWidget(renderingWidget_);

        renderingScene_ = new RenderingScene;
        renderingScene_->setCamera(getRenderingView()->getCamera());
        renderingScene_->setCameraManipulator(getRenderingView()->getCameraManipulator());
        getRenderingView()->setSceneData(renderingScene_->getRoot());
        renderingWidget_->setRenderingScene(renderingScene_);
    }

    createActions();

    std::cout << "libbsdf-" << lb::getVersion() << std::endl;
    std::cout << "OpenSceneGraph-" << osgGetVersion() << std::endl;
    std::cout << "Qt-" << qVersion() << std::endl;
    std::cout << "Eigen-" << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;    
}

MainWindow::~MainWindow()
{
    delete graphScene_;
    delete ui_;
}

void MainWindow::openBxdfUsingDialog()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open BxDF File"), QString(),
                                                    tr("BxDF Files (*.ddr *.ddt *.sdr *.sdt *.bsdf *.astm *.binary);;"
                                                       "Integra DDR Files (*.ddr);;"
                                                       "Integra DDT Files (*.ddt);;"
                                                       "Integra SDR Files (*.sdr);;"
                                                       "Integra SDR Files (*.sdt);;"
                                                       "LightTools/Zemax BSDF Files (*.bsdf);;"
                                                       "ASTM Files (*.astm);;"
                                                       "MERL binary Files (*.binary)"));
    
    if (fileName.isEmpty()) return;

    openFile(fileName);
}

void MainWindow::openFile(const QString& fileName)
{
    bool hadData = (graphScene_->getBrdf() ||
                    graphScene_->getBtdf() ||
                    graphScene_->getSpecularReflectances() ||
                    graphScene_->getSpecularTransmittances());

    bool loaded = false;

    lb::FileType fileType = lb::reader_utility::classifyFile(fileName.toLocal8Bit().data());
    switch (fileType) {
        case lb::ASTM_FILE: {
            loaded = openAstm(fileName);
            break;
        }
        case lb::INTEGRA_DDR_FILE: {
            loaded = openDdrDdt(fileName, lb::BRDF_DATA);
            break;
        }
        case lb::INTEGRA_DDT_FILE: {
            loaded = openDdrDdt(fileName, lb::BTDF_DATA);
            break;
        }
        case lb::INTEGRA_SDR_FILE: {
            loaded = openSdrSdt(fileName, lb::SPECULAR_REFLECTANCE_DATA);
            break;
        }
        case lb::INTEGRA_SDT_FILE: {
            loaded = openSdrSdt(fileName, lb::SPECULAR_TRANSMITTANCE_DATA);
            break;
        }
        case lb::LIGHTTOOLS_FILE: {
            loaded = openLightToolsBsdf(fileName);
            break;
        }
        case lb::ZEMAX_FILE: {
            loaded = openZemaxBsdf(fileName);
            break;
        }
        case lb::MERL_BINARY_FILE: {
            loaded = openMerlBinary(fileName);
            break;
        }
        default: {
            break;
        }
    }

    if (!loaded) {
        QMessageBox::warning(this, tr("BSDF Viewer"),
                             tr("Failed to load \"") + fileName + "\"",
                             QMessageBox::Ok);
        return;
    }

    if (!hadData) {
        viewFront();
    }

    QFileInfo fileInfo(fileName);
    this->setWindowTitle(fileInfo.fileName() + " - BSDF Viewer");
}

void MainWindow::exportBxdfUsingDialog()
{
    if (!graphScene_->getBrdf() && !graphScene_->getBtdf()) return;

    QString fileName = QFileDialog::getSaveFileName(this, tr("Export BxDF File"), QString(),
                                                    tr("Integra DDR Files (*.ddr);;"
                                                       "Integra DDT Files (*.ddt)"));
    
    if (fileName.isEmpty()) return;

    exportFile(fileName);
}

void MainWindow::exportFile(const QString& fileName)
{
    if (fileName.endsWith(".ddr")) {
        exportDdrDdt(fileName, lb::BRDF_DATA);
    }
    else if (fileName.endsWith(".ddt")) {
        exportDdrDdt(fileName, lb::BTDF_DATA);
    }
}

void MainWindow::viewFront()
{
    scene_util::fitCameraPosition(getMainView()->getCamera(),
                                  osg::Vec3(0.0, 1.0, 0.0),
                                  osg::Vec3(0.0, 0.0, 1.0),
                                  graphScene_->getBsdfGroup());
    updateCameraPosition();
}

void MainWindow::viewBack()
{
    scene_util::fitCameraPosition(getMainView()->getCamera(),
                                  osg::Vec3(0.0, -1.0, 0.0),
                                  osg::Vec3(0.0, 0.0, 1.0),
                                  graphScene_->getBsdfGroup());
    updateCameraPosition();
}

void MainWindow::viewRight()
{
    scene_util::fitCameraPosition(getMainView()->getCamera(),
                                  osg::Vec3(-1.0, 0.0, 0.0),
                                  osg::Vec3(0.0, 0.0, 1.0),
                                  graphScene_->getBsdfGroup());
    updateCameraPosition();
}

void MainWindow::viewLeft()
{
    scene_util::fitCameraPosition(getMainView()->getCamera(),
                                  osg::Vec3(1.0, 0.0, 0.0),
                                  osg::Vec3(0.0, 0.0, 1.0),
                                  graphScene_->getBsdfGroup());
    updateCameraPosition();
}

void MainWindow::viewTop()
{
    scene_util::fitCameraPosition(getMainView()->getCamera(),
                                  osg::Vec3(0.0, 0.0, 1.0),
                                  osg::Vec3(0.0, -1.0, 0.0),
                                  graphScene_->getBsdfGroup());
    updateCameraPosition();
}

void MainWindow::viewBottom()
{
    scene_util::fitCameraPosition(getMainView()->getCamera(),
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

void MainWindow::updateDisplayMode(QString modeName)
{
    ui_->incomingPolarAngleSlider->setEnabled(true);
    ui_->wavelengthSlider->setEnabled(true);
    ui_->pickedValueLineEdit->setEnabled(true);

    if (modeName == getDisplayModeName(GraphScene::NORMAL_DISPLAY)) {
        graphScene_->setDisplayMode(GraphScene::NORMAL_DISPLAY);
    }
    else if (modeName == getDisplayModeName(GraphScene::ALL_INCOMING_POLAR_ANGLES_DISPLAY)) {
        graphScene_->setDisplayMode(GraphScene::ALL_INCOMING_POLAR_ANGLES_DISPLAY);
        ui_->incomingPolarAngleSlider->setDisabled(true);
        ui_->pickedValueLineEdit->setDisabled(true);
    }
    else if (modeName == getDisplayModeName(GraphScene::ALL_WAVELENGTHS_DISPLAY)) {
        graphScene_->setDisplayMode(GraphScene::ALL_WAVELENGTHS_DISPLAY);
        ui_->wavelengthSlider->setDisabled(true);
        ui_->pickedValueLineEdit->setDisabled(true);
    }
    else if (modeName == getDisplayModeName(GraphScene::SAMPLE_POINTS_DISPLAY)) {
        graphScene_->setDisplayMode(GraphScene::SAMPLE_POINTS_DISPLAY);
    }
    else if (modeName == getDisplayModeName(GraphScene::SAMPLE_POINT_LABELS_DISPLAY)) {
        graphScene_->setDisplayMode(GraphScene::SAMPLE_POINT_LABELS_DISPLAY);
        ui_->pickedValueLineEdit->setDisabled(true);
    }
    else {
        std::cerr << "[MainWindow::updateDisplayMode] Undefined mode: " << modeName.toStdString() << std::endl;
        return;
    }

    graphScene_->updateGraphGeometry(ui_->incomingPolarAngleSlider->value(), 0, ui_->wavelengthSlider->value());
    graphScene_->updateView(graphWidget_->width(), graphWidget_->height());

    getMainView()->requestRedraw();

    clearPickedValue();
}

void MainWindow::updateIncomingPolarAngle(int index)
{
    graphScene_->updateGraphGeometry(index, 0, ui_->wavelengthSlider->value());

    float inPolarDegree = graphScene_->getIncomingPolarAngle(index) / lb::PI_F * 180.0f;
    ui_->incomingPolarAngleLineEdit->setText(QString::number(inPolarDegree));

    getMainView()->requestRedraw();

    clearPickedValue();
    displayReflectance();
}

void MainWindow::updateWavelength(int index)
{
    osg::Timer_t startTick = osg::Timer::instance()->tick();

    graphScene_->updateGraphGeometry(ui_->incomingPolarAngleSlider->value(), 0, index);
    getMainView()->requestRedraw();

    if (graphScene_->getColorModel() == lb::MONOCHROMATIC_MODEL &&
        graphScene_->getWavelength(index) == 0.0f) {
        ui_->wavelengthLabel->setText("Channel:");
        ui_->wavelengthLineEdit->clear();
    }
    else if (graphScene_->getColorModel() == lb::RGB_MODEL) {
        ui_->wavelengthLabel->setText("Channel:");
        switch (index) {
            case 0:  ui_->wavelengthLineEdit->setText("R"); break;
            case 1:  ui_->wavelengthLineEdit->setText("G"); break;
            case 2:  ui_->wavelengthLineEdit->setText("B"); break;
            default: assert(false);
        }
    }
    else if (graphScene_->getColorModel() == lb::XYZ_MODEL) {
        ui_->wavelengthLabel->setText("Channel:");
        switch (index) {
            case 0:  ui_->wavelengthLineEdit->setText("X"); break;
            case 1:  ui_->wavelengthLineEdit->setText("Y"); break;
            case 2:  ui_->wavelengthLineEdit->setText("Z"); break;
            default: assert(false);
        }
    }
    else {
        float wavelength = graphScene_->getWavelength(index);
        ui_->wavelengthLabel->setText("Wavelength:");
        ui_->wavelengthLineEdit->setText(QString::number(wavelength) + "nm");
    }

    clearPickedValue();
    displayReflectance();

    createTable();

    osg::Timer_t endTick = osg::Timer::instance()->tick();
    double delta = osg::Timer::instance()->delta_s(startTick, endTick);
    std::cout << "[scene_util::updateWavelength] " << delta << "(s)" << std::endl;
}

void MainWindow::useLogPlot(bool on)
{
    graphScene_->useLogPlot(on);
    updateBaseOfLogarithm(ui_->logPlotBaseSlider->value());
}

void MainWindow::updateBaseOfLogarithm(int index)
{
    graphScene_->setBaseOfLogarithm(index);
    graphScene_->updateGraphGeometry(ui_->incomingPolarAngleSlider->value(),
                                     0,
                                     ui_->wavelengthSlider->value());
    getMainView()->requestRedraw();

    ui_->logPlotBaseLineEdit->setText(QString::number(index));

    createTable();
}

void MainWindow::updateLightIntensity(double intensity)
{
    renderingScene_->setLightIntensity(intensity);
    renderingScene_->updateView(renderingWidget_->width(), renderingWidget_->height());
    getRenderingView()->requestRedraw();
}

void MainWindow::updateEnvironmentIntensity(double intensity)
{
    renderingScene_->setEnvironmentIntensity(intensity);
    renderingScene_->updateView(renderingWidget_->width(), renderingWidget_->height());
    getRenderingView()->getCamera()->setClearColor(osg::Vec4(intensity, intensity, intensity, 1.0));
    getRenderingView()->requestRedraw();
}

void MainWindow::displayPickedValue(const osg::Vec3& position)
{
    //std::cout << "[MainWindow::displayPickedValue] position: " << position << std::endl;

    if (!ui_->pickedValueLineEdit->isEnabled()) {
        ui_->pickedValueLineEdit->clear();
        return;
    }

    lb::Spectrum sp;

    if (position.z() > 0.0f) {
        lb::Brdf* brdf = graphScene_->getBrdf();
        lb::SampleSet2D* sr = graphScene_->getSpecularReflectances();
        if (brdf) {
            lb::Vec3 inDir = graphScene_->getInDir(ui_->incomingPolarAngleSlider->value(), 0);
            lb::Vec3 outDir = lb::toVec3(position).normalized();
            sp = brdf->getSpectrum(inDir, outDir);
        }
        else if (sr) {
            float inTheta = sr->getTheta(ui_->incomingPolarAngleSlider->value());
            float inPhi = sr->getPhi(0);
            lb::Vec3 inDir = lb::SphericalCoordinateSystem::toXyz(inTheta, inPhi);
            sp = sr->getSpectrum(inDir);
        }
        else {
            return;
        }
    }
    else {
        lb::Btdf* btdf = graphScene_->getBtdf();
        lb::SampleSet2D* st = graphScene_->getSpecularTransmittances();
        if (btdf) {
            lb::Vec3 inDir = graphScene_->getInDir(ui_->incomingPolarAngleSlider->value(), 0);
            lb::Vec3 outDir = lb::toVec3(position).normalized();
            sp = btdf->getSpectrum(inDir, outDir);
        }
        else if (st) {
            float inTheta = st->getTheta(ui_->incomingPolarAngleSlider->value());
            float inPhi = st->getPhi(0);
            lb::Vec3 inDir = lb::SphericalCoordinateSystem::toXyz(inTheta, inPhi);
            sp = st->getSpectrum(inDir);
        }
        else {
            return;
        }
    }

    float pickedValue = sp[ui_->wavelengthSlider->value()];
    ui_->pickedValueLineEdit->setText(QString::number(pickedValue));
}

void MainWindow::clearPickedValue()
{
    ui_->pickedValueLineEdit->clear();
}

void MainWindow::displayReflectance()
{
    lb::SampleSet2D* ss2;

    if (graphScene_->getBrdf()) {
        ss2 = graphScene_->getReflectances();
        ui_->pickedReflectanceLabel->setText(tr("Reflectance:"));
    }
    else if (graphScene_->getBtdf()) {
        ss2 = graphScene_->getReflectances();
        ui_->pickedReflectanceLabel->setText(tr("Transmittance:"));
    }
    else if (graphScene_->getSpecularReflectances()) {
        ss2 = graphScene_->getSpecularReflectances();
        ui_->pickedReflectanceLabel->setText(tr("Reflectance:"));
    }
    else if (graphScene_->getSpecularTransmittances()) {
        ss2 = graphScene_->getSpecularTransmittances();
        ui_->pickedReflectanceLabel->setText(tr("Transmittance:"));
    }
    else {
        return;
    }

    const lb::Spectrum& sp = ss2->getSpectrum(ui_->incomingPolarAngleSlider->value());
    float reflectance = sp[ui_->wavelengthSlider->value()];
    ui_->pickedReflectanceLineEdit->setText(QString::number(reflectance));
}

void MainWindow::createActions()
{
    ui_->viewMenu->addAction(ui_->controlDockWidget->toggleViewAction());
    ui_->viewMenu->addAction(ui_->renderingDockWidget->toggleViewAction());
    ui_->viewMenu->addAction(ui_->informationDockWidget->toggleViewAction());
    ui_->viewMenu->addAction(ui_->tableDockWidget->toggleViewAction());

    ui_->tableDockWidget->toggleViewAction()->setShortcut(Qt::CTRL + Qt::Key_T);

    connect(ui_->actionOpenBrdf,    SIGNAL(triggered()), this, SLOT(openBxdfUsingDialog()));
    connect(ui_->actionExport,      SIGNAL(triggered()), this, SLOT(exportBxdfUsingDialog()));
    connect(ui_->actionQuit,        SIGNAL(triggered()), this, SLOT(close()));

    connect(ui_->actionViewFront,   SIGNAL(triggered()), this, SLOT(viewFront()));
    connect(ui_->actionViewBack,    SIGNAL(triggered()), this, SLOT(viewBack()));
    connect(ui_->actionViewRight,   SIGNAL(triggered()), this, SLOT(viewRight()));
    connect(ui_->actionViewLeft,    SIGNAL(triggered()), this, SLOT(viewLeft()));
    connect(ui_->actionViewTop,     SIGNAL(triggered()), this, SLOT(viewTop()));
    connect(ui_->actionViewBottom,  SIGNAL(triggered()), this, SLOT(viewBottom()));

    connect(ui_->actionAbout, SIGNAL(triggered()), this, SLOT(about()));

    connect(graphWidget_, SIGNAL(fileDropped(QString)), this, SLOT(openFile(QString)));
    connect(graphWidget_, SIGNAL(picked(osg::Vec3)),    this, SLOT(displayPickedValue(osg::Vec3)));
    connect(graphWidget_, SIGNAL(clearPickedValue()),   this, SLOT(clearPickedValue()));
    connect(graphWidget_, SIGNAL(viewFront()),          this, SLOT(viewFront()));
}

void MainWindow::initializeUi()
{
    this->setWindowTitle("BSDF Viewer");

    QComboBox* comboBox = ui_->displayModeComboBox;

    comboBox->clear();
    comboBox->addItem(getDisplayModeName(GraphScene::NORMAL_DISPLAY));
    comboBox->addItem(getDisplayModeName(GraphScene::ALL_INCOMING_POLAR_ANGLES_DISPLAY));
    comboBox->addItem(getDisplayModeName(GraphScene::ALL_WAVELENGTHS_DISPLAY));
    comboBox->addItem(getDisplayModeName(GraphScene::SAMPLE_POINTS_DISPLAY));
    comboBox->addItem(getDisplayModeName(GraphScene::SAMPLE_POINT_LABELS_DISPLAY));

    ui_->incomingPolarAngleSlider->setEnabled(true);
    ui_->incomingPolarAngleSlider->setMaximum(graphScene_->getNumInTheta() - 1);
    ui_->incomingPolarAngleSlider->setValue(0);
    updateIncomingPolarAngle(0);

    ui_->wavelengthSlider->setEnabled(true);
    ui_->wavelengthSlider->setMaximum(graphScene_->getNumWavelengths() - 1);
    ui_->wavelengthSlider->setValue(0);
    updateWavelength(0);

    ui_->pickedValueLineEdit->setEnabled(true);

    if (graphScene_->getNumWavelengths() == 1) {
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::ALL_WAVELENGTHS_DISPLAY)));
    }

    if (graphScene_->getNumInTheta() == 1) {
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::ALL_INCOMING_POLAR_ANGLES_DISPLAY)));
    }

    if (!graphScene_->isInDirDependentCoordinateSystem()) {
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::SAMPLE_POINTS_DISPLAY)));
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::SAMPLE_POINT_LABELS_DISPLAY)));
    }

    if (graphScene_->getSpecularReflectances() ||
        graphScene_->getSpecularTransmittances()) {
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::ALL_INCOMING_POLAR_ANGLES_DISPLAY)));
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::ALL_WAVELENGTHS_DISPLAY)));
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::SAMPLE_POINTS_DISPLAY)));
        comboBox->removeItem(comboBox->findText(getDisplayModeName(GraphScene::SAMPLE_POINT_LABELS_DISPLAY)));
    }

    osgGA::TrackballManipulator* trackball = dynamic_cast<osgGA::TrackballManipulator*>(getMainView()->getCameraManipulator());
    if (trackball) {
        osg::Vec3d eye, center, up;
        getMainView()->getCamera()->getViewMatrixAsLookAt(eye, center, up);
        trackball->setHomePosition(eye, osg::Vec3d(0.0, 0.0, 0.0), up);
        trackball->home(0.0);
    }

    graphScene_->useOit(false);
    graphScene_->updateView(graphWidget_->width(), graphWidget_->height());
    getMainView()->requestRedraw();

    if (renderingScene_->getBrdf()) {
        ui_->lightIntensitySpinBox->setEnabled(true);
        ui_->lightIntensitySpinBox->setValue(1.0);
        ui_->environmentIntensitySpinBox->setValue(0.0);
    }
    else if (renderingScene_->getReflectance()) {
        ui_->lightIntensitySpinBox->setValue(0.0);
        ui_->lightIntensitySpinBox->setDisabled(true);
        ui_->environmentIntensitySpinBox->setValue(1.0);
    }

    renderingScene_->updateView(renderingWidget_->width(), renderingWidget_->height());
    getRenderingView()->requestRedraw();
}

QString MainWindow::getDisplayModeName(GraphScene::DisplayMode mode)
{
    switch (mode) {
        case GraphScene::NORMAL_DISPLAY: {
            return "Normal";
            break;
        }
        case GraphScene::ALL_INCOMING_POLAR_ANGLES_DISPLAY: {
            return "All incoming polar angles";
            break;
        }
        case GraphScene::ALL_WAVELENGTHS_DISPLAY: {
            bool isSpectral = (graphScene_->getColorModel() == lb::SPECTRAL_MODEL);
            return isSpectral ? "All wavelengths" : "All channels";
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

bool MainWindow::openDdrDdt(const QString& fileName, lb::DataType dataType)
{
    lb::SpecularCoordinatesBrdf* brdf = lb::DdrReader::read(fileName.toLocal8Bit().data());
    if (!brdf) return false;

    setupBrdf(brdf, dataType);

    return true;
}

bool MainWindow::openSdrSdt(const QString& fileName, lb::DataType dataType)
{
    lb::SampleSet2D* ss2 = lb::SdrReader::read(fileName.toLocal8Bit().data());
    if (!ss2) return false;

    graphScene_->clearData();

    if (dataType == lb::SPECULAR_REFLECTANCE_DATA) {
        graphScene_->setSpecularReflectances(ss2);
    }
    else if (dataType == lb::SPECULAR_TRANSMITTANCE_DATA) {
        graphScene_->setSpecularTransmittances(ss2);
    }
    else {
        std::cerr << "[MainWindow::openSdrSdt] Invalid data type: " << dataType << std::endl;
        return false;
    }

    renderingScene_->setData(0, ss2, dataType);

    initializeUi();

    ui_->logPlotGroupBox->setEnabled(false);
    ui_->tableGraphicsView->fitView(0.9);

    return true;
}

bool MainWindow::openLightToolsBsdf(const QString& fileName)
{
    lb::TwoSidedMaterial* material = lb::LightToolsBsdfReader::read(fileName.toLocal8Bit().data());
    if (!material) return false;

    OpenLightToolsBsdfDialog dialog(this);

    if (material->getFrontMaterial()->getBsdf()->getBrdf()) { dialog.addFrontBrdfItem(); }
    if (material->getFrontMaterial()->getBsdf()->getBtdf()) { dialog.addFrontBtdfItem(); }
    if (material->getBackMaterial()->getBsdf()->getBrdf())  { dialog.addBackBrdfItem(); }
    if (material->getBackMaterial()->getBsdf()->getBtdf())  { dialog.addBackBtdfItem(); }

    if (dialog.hasMultipleItems()) {
        if (dialog.exec() == QDialog::Rejected) {
            delete material;
            return false;
        }
    }
    
    lb::Brdf* brdf;
    lb::DataType dataType;
    if (dialog.isFrontBrdf()) {
        brdf = material->getFrontMaterial()->getBsdf()->getBrdf();
        dataType = lb::BRDF_DATA;
    }
    else if (dialog.isFrontBtdf()) {
        brdf = material->getFrontMaterial()->getBsdf()->getBtdf()->getBrdf();
        dataType = lb::BTDF_DATA;
    }
    else if (dialog.isBackBrdf()) {
        brdf = material->getBackMaterial()->getBsdf()->getBrdf();
        dataType = lb::BRDF_DATA;
    }
    else if (dialog.isBackBtdf()) {
        brdf = material->getBackMaterial()->getBsdf()->getBtdf()->getBrdf();
        dataType = lb::BTDF_DATA;
    }
    else {
        return false;
    }

    lb::SphericalCoordinatesBrdf* scBrdf = dynamic_cast<lb::SphericalCoordinatesBrdf*>(brdf);
    setupBrdf(new lb::SphericalCoordinatesBrdf(*scBrdf), dataType);

    delete material;

    return true;
}

bool MainWindow::openZemaxBsdf(const QString& fileName)
{
    lb::DataType dataType;
    lb::SpecularCoordinatesBrdf* brdf = lb::ZemaxBsdfReader::read(fileName.toLocal8Bit().data(), &dataType);
    if (!brdf) return false;

    setupBrdf(brdf, dataType);

    return true;
}

bool MainWindow::openAstm(const QString& fileName)
{
    OpenAstmDialog dialog(this);
    if (dialog.exec() == QDialog::Rejected) return false;

    lb::SphericalCoordinatesBrdf* brdf = lb::AstmReader::read(fileName.toLocal8Bit().data());
    if (!brdf) return false;

    setupBrdf(brdf, dialog.getDataType());

    return true;
}

bool MainWindow::openMerlBinary(const QString& fileName)
{
    lb::HalfDifferenceCoordinatesBrdf* brdf = lb::MerlBinaryReader::read(fileName.toLocal8Bit().data());
    if (!brdf) return false;

    setupBrdf(brdf);

    return true;
}

void MainWindow::setupBrdf(lb::Brdf* brdf, lb::DataType dataType)
{
    graphScene_->clearData();
    if (dataType == lb::BRDF_DATA) {
        graphScene_->setBrdf(brdf);
    }
    else if (dataType == lb::BTDF_DATA) {
        graphScene_->setBtdf(new lb::Btdf(brdf));
    }
    else {
        std::cerr << "[MainWindow::setupBrdf] Invalid data type: " << dataType << std::endl;
        return;
    }
    graphScene_->createBrdfGeode();

    renderingScene_->setData(brdf, graphScene_->getReflectances(), dataType);

    initializeUi();

    ui_->logPlotGroupBox->setEnabled(true);
    ui_->tableGraphicsView->fitView(0.9);
}

void MainWindow::exportDdrDdt(const QString& fileName, lb::DataType dataType)
{
    lb::Brdf* brdf;
    if (dataType == lb::BRDF_DATA && graphScene_->getBrdf()) {
        brdf = graphScene_->getBrdf();
    }
    else if (dataType == lb::BTDF_DATA && graphScene_->getBtdf()) {
        brdf = graphScene_->getBtdf()->getBrdf();
    }
    else {
        std::cerr << "[MainWindow::exportDdrDdt] Invalid data for export." << std::endl;
        return;
    }

    lb::SpecularCoordinatesBrdf* exportedBrdf;
    if (dynamic_cast<lb::SpecularCoordinatesBrdf*>(brdf)) {
        exportedBrdf = new lb::SpecularCoordinatesBrdf(dynamic_cast<const lb::SpecularCoordinatesBrdf&>(*brdf));
    }
    else if (!graphScene_->isInDirDependentCoordinateSystem()) {
        exportedBrdf = new lb::SpecularCoordinatesBrdf(*brdf, 10, 1, 181, 37);
    }
    else {
        const lb::SampleSet* ss = brdf->getSampleSet();
        lb::Arrayf inThetaAngles  = ss->getAngles0();
        lb::Arrayf inPhiAngles    = lb::Arrayf::LinSpaced(ss->getNumAngles1(), 0.0, lb::SpecularCoordinateSystem::MAX_ANGLE1);
        lb::Arrayf outThetaAngles = lb::Arrayf::LinSpaced(181,                 0.0, lb::SpecularCoordinateSystem::MAX_ANGLE2);
        lb::Arrayf outPhiAngles   = lb::Arrayf::LinSpaced(37,                  0.0, lb::SpecularCoordinateSystem::MAX_ANGLE3);

        if (inPhiAngles.size() == 1) {
            inPhiAngles[0] = 0.0f;
        }

        exportedBrdf = new lb::SpecularCoordinatesBrdf(*brdf, inThetaAngles, inPhiAngles, outThetaAngles, outPhiAngles);
    }

    lb::SampleSet* exportedSs = exportedBrdf->getSampleSet();
    if (exportedSs->getColorModel() == lb::XYZ_MODEL) {
        exportedSs->convertFromXyzToSrgb();
    }
    exportedBrdf->expand();
    exportedBrdf->fixEnergyConservation();

    lb::DdrWriter::write(fileName.toLocal8Bit().data(), *exportedBrdf);
    delete exportedBrdf;
}

void MainWindow::updateCameraPosition()
{
    osgGA::CameraManipulator* cm = getMainView()->getCameraManipulator();
    if (cm) {
        osg::Vec3d eye, center, up;
        getMainView()->getCamera()->getViewMatrixAsLookAt(eye, center, up);
        cm->setHomePosition(eye, osg::Vec3d(0.0, 0.0, 0.0), up);
        cm->home(0.0);
    }

    getMainView()->requestRedraw();
}

void MainWindow::createTable()
{
    float gamma = ui_->logPlotGroupBox->isChecked() ? ui_->logPlotBaseSlider->value() : 1.0f;
    ui_->tableGraphicsView->createTable(ui_->wavelengthSlider->value(), gamma);
}
