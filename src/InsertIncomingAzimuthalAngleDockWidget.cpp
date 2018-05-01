// =================================================================== //
// Copyright (C) 2018 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "InsertIncomingAzimuthalAngleDockWidget.h"

#include <QtWidgets>

#include <libbsdf/Brdf/Processor.h>

#include <libbsdf/Reader/AstmReader.h>
#include <libbsdf/Reader/DdrReader.h>
#include <libbsdf/Reader/ReaderUtility.h>

InsertIncomingAzimuthalAngleDockWidget::InsertIncomingAzimuthalAngleDockWidget(
    QWidget* parent)
    : QDockWidget(parent),
      baseBrdf_(0),
      insertedBrdf_(0),
      ui_(new Ui::InsertIncomingAzimuthalAngleDockWidgetBase)
{
    ui_->setupUi(this);

    connect(ui_->filePushButton,    SIGNAL(clicked()), this, SLOT(setFileName()));
    connect(ui_->processPushButton, SIGNAL(clicked()), this, SLOT(process()));
}

InsertIncomingAzimuthalAngleDockWidget::~InsertIncomingAzimuthalAngleDockWidget()
{
    delete ui_;
}

void InsertIncomingAzimuthalAngleDockWidget::setBrdf(lb::Brdf* brdf)
{
    baseBrdf_ = brdf;
}

void InsertIncomingAzimuthalAngleDockWidget::setFileName()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open BRDF/BTDF File"), QString(),
//                                                    tr("BxDF Files (*.ddr *.ddt *.astm);;"
//                                                       "Integra DDR Files (*.ddr);;"
//                                                       "Integra DDT Files (*.ddt);;"
//                                                       "ASTM Files (*.astm)"
//                                                       ));
                                                    tr("ASTM Files (*.astm)"));

    if (fileName.isEmpty()) return;

    ui_->fileNameLineEdit->setText(fileName);
    ui_->fileNameLineEdit->setToolTip(fileName);
}

void InsertIncomingAzimuthalAngleDockWidget::process()
{
    openFile(ui_->fileNameLineEdit->text());

    if (!baseBrdf_ || !insertedBrdf_) return;

    typedef lb::SphericalCoordinatesBrdf SpheBrdf;
    SpheBrdf* baseBrdf = dynamic_cast<SpheBrdf*>(baseBrdf_);
    SpheBrdf* insertedBrdf = dynamic_cast<SpheBrdf*>(insertedBrdf_);

    if (!baseBrdf || !insertedBrdf) {
        QMessageBox::warning(this, tr("BSDF Processor"),
                             tr("Unsupported type of BRDF for insertion") + ui_->fileNameLineEdit->text() + "\"",
                             QMessageBox::Ok);
        return;
    }

    float inPhi = lb::toRadian(ui_->angleDoubleSpinBox->value());
    lb::Brdf* brdf = lb::insertBrdfAlongInPhi(*baseBrdf, *insertedBrdf, inPhi);

    if (brdf) {
        ui_->fileNameLineEdit->clear();

        emit processed(brdf);
    }
    else {
        QMessageBox::warning(this, tr("BSDF Processor"),
                             tr("Failed to insert \"") + ui_->fileNameLineEdit->text() + "\"",
                             QMessageBox::Ok);
        return;
    }
}

void InsertIncomingAzimuthalAngleDockWidget::openFile(const QString& fileName)
{
    lb::FileType fileType = lb::reader_utility::classifyFile(fileName.toLocal8Bit().data());
    switch (fileType) {
        case lb::ASTM_FILE: {
            insertedBrdf_ = lb::AstmReader::read(fileName.toLocal8Bit().data());
            break;
        }
        //case lb::INTEGRA_DDR_FILE:
        //case lb::INTEGRA_DDT_FILE: {
        //    insertedBrdf_ = lb::DdrReader::read(fileName.toLocal8Bit().data());
        //    break;
        //}
        default: {
            QMessageBox::warning(this, tr("BSDF Processor"),
                                 tr("Unsupported type of BRDF: \"") + ui_->fileNameLineEdit->text() + "\"",
                                 QMessageBox::Ok);
            break;
        }
    }
}
