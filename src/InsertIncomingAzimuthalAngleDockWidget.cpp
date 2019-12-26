// =================================================================== //
// Copyright (C) 2018-2019 Kimura Ryo                                  //
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
    QString fileName = QFileDialog::getOpenFileName(this, "Open BRDF/BTDF File", QString(), "ASTM Files (*.astm)");

    if (fileName.isEmpty()) return;

    ui_->fileNameLineEdit->setText(fileName);
    ui_->fileNameLineEdit->setToolTip(fileName);
}

void InsertIncomingAzimuthalAngleDockWidget::process()
{
    openFile(ui_->fileNameLineEdit->text());

    if (!baseBrdf_ || !insertedBrdf_) return;

    auto baseBrdf     = dynamic_cast<lb::SphericalCoordinatesBrdf*>(baseBrdf_);
    auto insertedBrdf = dynamic_cast<lb::SphericalCoordinatesBrdf*>(insertedBrdf_);

    if (!baseBrdf || !insertedBrdf) {
        QMessageBox::warning(this, qApp->applicationName(),
                             "Unsupported type of BRDF for insertion" + ui_->fileNameLineEdit->text() + "\"");
        return;
    }

    float inPhi = lb::toRadian(ui_->angleDoubleSpinBox->value());
    lb::Brdf* brdf = lb::insertBrdfAlongInPhi(*baseBrdf, *insertedBrdf, inPhi);

    if (brdf) {
        ui_->fileNameLineEdit->clear();

        emit processed(std::shared_ptr<lb::Brdf>(brdf));
    }
    else {
        QMessageBox::warning(this, qApp->applicationName(),
                             "Failed to insert \"" + ui_->fileNameLineEdit->text() + "\"");
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
        default: {
            QMessageBox::warning(this, qApp->applicationName(),
                                 "Unsupported type of BRDF: \"" + ui_->fileNameLineEdit->text() + "\"");
            break;
        }
    }
}
