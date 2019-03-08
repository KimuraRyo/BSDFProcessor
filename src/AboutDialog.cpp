// =================================================================== //
// Copyright (C) 2015 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "AboutDialog.h"

#include <osg/Version>

#include <Eigen/Core>

#include <libbsdf/Common/Version.h>

#include "Version.h"
#include "LicenseInformationDialog.h"

AboutDialog::AboutDialog(QWidget* parent) : QDialog(parent),
                                            ui_(new Ui::AboutDialogBase)
{
    ui_->setupUi(this);

    setWindowFlags(Qt::Dialog | Qt::MSWindowsFixedSizeDialogHint);

    ui_->versionLabel->setText("Version " + QString(getVersion()));

    ui_->libbsdfLabel->setText("libbsdf-" + QString(lb::getVersion()));
    ui_->qtLabel->setText("Qt-" + QString(qVersion()));
    ui_->osgLabel->setText("OpenSceneGraph-" + QString(osgGetVersion()));

    std::stringstream eigenStream;
    eigenStream << "Eigen-" << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION;
    std::string eigenVerStr(eigenStream.str());
    ui_->eigenLabel->setText(eigenVerStr.c_str());
}

AboutDialog::~AboutDialog()
{
    delete ui_;
}

void AboutDialog::showLicenseInformation()
{
    LicenseInformationDialog dialog(this);
    dialog.exec();
}
