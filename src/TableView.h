// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef TABLE_VIEW_H
#define TABLE_VIEW_H

#include <QtWidgets/QGraphicsView>

class GraphScene;

namespace lb
{
    class SampleSet;
}

/*!
 * \class   TableView
 * \brief   The TableView class provides the table view of sample points.
 */
class TableView : public QGraphicsView
{
    Q_OBJECT
    
public:
    explicit TableView(QWidget* parent = 0);

    void createTable(int spectrumIndex, float gamma = 1.0f);

    void setGraphScene(GraphScene* scene) { graphScene_ = scene; }

public slots:
    void fitView(qreal scaleFactor = 1.0)
    {
        fitInView(scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
        scale(scaleFactor, scaleFactor);
    }

private:
    Q_DISABLE_COPY(TableView)

    void createBrdfTable(int spectrumIndex, float gamma = 1.0f);
    void createBrdfDataItems(const lb::SampleSet* ss, int spectrumIndex, float gamma = 1.0f);
    void createBrdfDataPixmapItem(const lb::SampleSet* ss, int spectrumIndex, float gamma = 1.0f);
    void createBrdfAngleItems(const lb::SampleSet* ss);

    void createReflectanceTable(int spectrumIndex, float gamma = 1.0f);

    void addAngleItem(const QColor& color, float angle,
                      qreal posX, qreal posY,
                      qreal lodThreshold = 25.0, qreal textLodThreshold = 25.0);

    void paintEvent(QPaintEvent* event);
    void wheelEvent(QWheelEvent* event);
    void contextMenuEvent(QContextMenuEvent* event);

    QGraphicsScene* graphicsScene_;

    QAction* actionFitView_;

    GraphScene* graphScene_;
};

#endif // TABLE_VIEW_H
