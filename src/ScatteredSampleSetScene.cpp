// =================================================================== //
// Copyright (C) 2022 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "ScatteredSampleSetScene.h"

ScatteredSampleSetScene::ScatteredSampleSetScene(QObject *parent) : QGraphicsScene(parent)
{
}

void ScatteredSampleSetScene::mouseMoveEvent(QGraphicsSceneMouseEvent* event)
{
    QGraphicsScene::mouseMoveEvent(event);

    emit mouseMoved(event->scenePos());
}

void ScatteredSampleSetScene::mouseReleaseEvent(QGraphicsSceneMouseEvent* event)
{
    QGraphicsScene::mouseReleaseEvent(event);

    if (event->button() == Qt::LeftButton &&
        event->screenPos() == event->lastScreenPos()) {
        emit mouseClicked(event->scenePos());
    }
}
