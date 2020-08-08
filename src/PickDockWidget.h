// =================================================================== //
// Copyright (C) 2020 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef INFORMATION_DOCKWIDGET_H
#define INFORMATION_DOCKWIDGET_H

#include <QtWidgets>

#include "ui_PickDockWidget.h"

#include "GraphScene.h"
#include "MaterialData.h"

/*!
 * \class   PickDockWidget
 * \brief   The PickDockWidget class provides the dock widget to show information about the picked direction.
 */
class PickDockWidget : public QDockWidget
{
    Q_OBJECT

public:
    explicit PickDockWidget(QWidget* parent);
    ~PickDockWidget();

    void setGraphScene(GraphScene* scene) { graphScene_ = scene; }
    void setMaterialData(MaterialData* materialData) { data_ = materialData; }

    void updatePickedAngle(const lb::Vec3& inDir, const lb::Vec3& outDir, bool isotropic);
    void updatePickedValue(const lb::Vec3& inDir, const lb::Vec3& outDir);
    void updatePickedValue();

    void displayReflectance();

    void clearPickedValueLineEdit() { ui_->pickedValueLineEdit->clear(); }
    void enablePickedValueLineEdit(bool enabled) { ui_->pickedValueLineEdit->setEnabled(enabled); }

    void clearPickedReflectanceLineEdit() { ui_->pickedReflectanceLineEdit->clear(); }
    void enablePickedReflectanceLineEdit(bool enabled) { ui_->pickedReflectanceLineEdit->setEnabled(enabled); }

signals:
    void redrawGraphRequested();

public slots:
    void updateOutDir(const osg::Vec3& dir);
    void clearPickedValue();

private slots:
    void displayArcsInGraph(int index);
    void useArcsInGraph(bool on);
    void copyInfo();

private:
    Q_DISABLE_COPY(PickDockWidget)

    void contextMenuEvent(QContextMenuEvent* event) override;

    GraphScene* graphScene_;
    MaterialData* data_;

    QAction* actionCopyInfo_;

    Ui::PickDockWidgetBase* ui_;
};

#endif // INFORMATION_DOCKWIDGET_H
