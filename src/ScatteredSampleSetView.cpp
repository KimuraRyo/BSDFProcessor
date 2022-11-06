// =================================================================== //
// Copyright (C) 2022 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "ScatteredSampleSetView.h"

#include <QtWidgets>

#include <libbsdf/Common/DelaunayTriangulation.h>
#include <libbsdf/Common/Global.h>
#include <libbsdf/Common/Log.h>
#include <libbsdf/Common/StereographicProjection.h>

#include "Utility.h"

constexpr int ITEM_SIZE = 1.0;
constexpr qreal ALWAYS_VISIBLE_LOD = 0.0;

ScatteredSampleSetView::ScatteredSampleSetView(QWidget* parent)
    : QGraphicsView(parent), fittingNeeded_(true)
{
    actionFitView_ = new QAction(this);
    actionFitView_->setText("Fit in view");
    connect(actionFitView_, SIGNAL(triggered()), this, SLOT(fitView()));

    sssScene_ = new ScatteredSampleSetScene;
    setScene(sssScene_);
}

void ScatteredSampleSetView::setScatteredSampleSet2D(lb::ScatteredSampleSet2D* sss2d)
{
    sss2d_ = sss2d;

    initializeScene();
    addDelaunayTriangles();
    addPoints();
}

void ScatteredSampleSetView::fitView(qreal scaleFactor)
{
    fitInView(scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
    scale(scaleFactor, scaleFactor);
}

void ScatteredSampleSetView::paintEvent(QPaintEvent* event)
{
    QGraphicsView::paintEvent(event);
}

void ScatteredSampleSetView::wheelEvent(QWheelEvent* event)
{
    if (event->angleDelta().y() > 0) {
        scale(0.9, 0.9);
    }
    else {
        scale(1.0 / 0.9, 1.0 / 0.9);
    }
    event->accept();
}

void ScatteredSampleSetView::contextMenuEvent(QContextMenuEvent* event)
{
    QMenu menu(this);
    menu.addAction(actionFitView_);

    menu.exec(event->globalPos());
}

void ScatteredSampleSetView::showEvent(QShowEvent* event)
{
    Q_UNUSED(event);

    // Initialize the scale of this view with scene.
    // If data is set before the view is shown, the proper size of the view is not acquired.
    if (fittingNeeded_) {
        fitView(0.9);
    }
    fittingNeeded_ = false;
}

void ScatteredSampleSetView::initializeScene()
{
    scene()->clear();

    const qreal sceneSize = 100.0;
    scene()->setSceneRect(-sceneSize / 2.0, -sceneSize / 2.0, sceneSize, sceneSize);

    QPen pen;
    pen.setWidthF(0.002);
    pen.setJoinStyle(Qt::MiterJoin);
    scene()->addEllipse(-1, -1, 2, 2, pen);
}

void ScatteredSampleSetView::addPoints()
{
    QPen pen(Qt::NoPen);
    pen.setCapStyle(Qt::RoundCap);

    QBrush brush(QColor(50, 200, 50), Qt::SolidPattern);

    for (const auto& s : sss2d_->getSampleMap()) {
        // All points are inverted for visibility.
        lb::Vec2 pos = -s.first;

        constexpr double radius = 0.003;
        QRectF           ellipseRect(pos[0] - radius, pos[1] - radius, radius * 2, radius * 2);
        QGraphicsEllipseItem* item = scene()->addEllipse(ellipseRect, pen, brush);

        lb::Spectrum sp = s.second;
        item->setToolTip(QString::number(sp[0]));
    }
}

void ScatteredSampleSetView::addDelaunayTriangles()
{
    const lb::DelaunayTriangulation& dt = sss2d_->getDelaunayTriangulation();
    const std::vector<double>&       coords = dt.getCoords();

    QPen pen(QColor(255, 0, 255));
    pen.setWidthF(0.002);
    pen.setCapStyle(Qt::RoundCap);

    // Find and add all edges.
    std::set<std::set<size_t>> edges;
    for (size_t i = 0; i < dt.getNumTriangles(); ++i) {
        lb::Vec3i trg(dt.getTriangle(i));

        size_t index0 = trg[0];
        size_t index1 = trg[1];
        size_t index2 = trg[2];

        // All points are inverted for visibility.
        lb::Vec2 vert0(-dt.getVertex(index0));
        lb::Vec2 vert1(-dt.getVertex(index1));
        lb::Vec2 vert2(-dt.getVertex(index2));

        QPointF p0(vert0[0], vert0[1]);
        QPointF p1(vert1[0], vert1[1]);
        QPointF p2(vert2[0], vert2[1]);

        std::set<size_t> edge0{index0, index1};
        std::set<size_t> edge1{index1, index2};
        std::set<size_t> edge2{index2, index0};

        auto ret0 = edges.insert(edge0);
        auto ret1 = edges.insert(edge1);
        auto ret2 = edges.insert(edge2);

        if (ret0.second) {
            scene()->addLine(QLineF(p0, p1), pen);
        }

        if (ret1.second) {
            scene()->addLine(QLineF(p1, p2), pen);
        }

        if (ret2.second) {
            scene()->addLine(QLineF(p2, p0), pen);
        }
    }
}
