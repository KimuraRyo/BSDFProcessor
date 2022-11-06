// =================================================================== //
// Copyright (C) 2022 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef SCATTERED_SAMPLE_SET_SCENE_H
#define SCATTERED_SAMPLE_SET_SCENE_H

#include <QtWidgets/QGraphicsScene>
#include <QGraphicsSceneMouseEvent>

/*!
 * \class   ScatteredSampleSetScene
 * \brief   The ScatteredSampleSetScene class provides the scene of 2D scattered sample set view.
 */
class ScatteredSampleSetScene : public QGraphicsScene
{
    Q_OBJECT

public:
    explicit ScatteredSampleSetScene(QObject* parent = nullptr);

signals:
    void mouseMoved(QPointF pos);
    void mouseClicked(QPointF pos);
    void mouseDoubleClicked(QPointF pos);

private:
    Q_DISABLE_COPY(ScatteredSampleSetScene)

    void mouseMoveEvent(QGraphicsSceneMouseEvent* event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event);
};

#endif // SCATTERED_SAMPLE_SET_VIEW_H
