// =================================================================== //
// Copyright (C) 2022 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef SCATTERED_SAMPLE_SET_VIEW_H
#define SCATTERED_SAMPLE_SET_VIEW_H

#include <QtWidgets/QGraphicsView>

#include <libbsdf/Brdf/ScatteredSampleSet2D.h>

#include "ScatteredSampleSetScene.h"

/*!
 * \class   ScatteredSampleSetView
 * \brief   The ScatteredSampleSetView class provides the view of 2D scattered sample set.
 */
class ScatteredSampleSetView : public QGraphicsView
{
    Q_OBJECT

public:
    explicit ScatteredSampleSetView(QWidget* parent = nullptr);

    void setScatteredSampleSet2D(lb::ScatteredSampleSet2D* sss2d);

    void initializeScene();

public slots:
    void fitView(qreal scaleFactor = 1.0);

private:
    Q_DISABLE_COPY(ScatteredSampleSetView)

    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void contextMenuEvent(QContextMenuEvent* event) override;
    void showEvent(QShowEvent* event) override;

    void addPoints();
    void addDelaunayTriangles();

    ScatteredSampleSetScene* sssScene_;

    QAction* actionFitView_;

    lb::ScatteredSampleSet2D* sss2d_;

    bool fittingNeeded_;
};

#endif // SCATTERED_SAMPLE_SET_VIEW_H
