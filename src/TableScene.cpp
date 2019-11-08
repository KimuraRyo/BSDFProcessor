// =================================================================== //
// Copyright (C) 2019 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "TableScene.h"

TableScene::TableScene(QObject *parent) : QGraphicsScene(parent)
{
}

void TableScene::mouseMoveEvent(QGraphicsSceneMouseEvent* event)
{
    QGraphicsScene::mouseMoveEvent(event);

    emit mouseMoved(event->scenePos());
}

void TableScene::mouseReleaseEvent(QGraphicsSceneMouseEvent* event)
{
    QGraphicsScene::mouseReleaseEvent(event);

    if (event->button() == Qt::LeftButton &&
        event->screenPos() == event->lastScreenPos()) {
        emit mouseClicked(event->scenePos());
    }
}

void TableScene::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event)
{
    QGraphicsScene::mouseDoubleClickEvent(event);

    if (event->button() == Qt::LeftButton) {
        emit mouseDoubleClicked(event->scenePos());
    }
}
