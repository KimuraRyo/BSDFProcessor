// =================================================================== //
// Copyright (C) 2019 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef TABLE_SCENE_H
#define TABLE_SCENE_H

#include <QtWidgets/QGraphicsScene>
#include <QGraphicsSceneMouseEvent>

/*!
 * \class   TableScene
 * \brief   The TableScene class provides the scene of table view.
 */
class TableScene : public QGraphicsScene
{
    Q_OBJECT

public:
    explicit TableScene(QObject *parent = nullptr);

signals:
    void mouseMoved(QPointF pos);
    void mouseClicked(QPointF pos);
    void mouseDoubleClicked(QPointF pos);

private:
    Q_DISABLE_COPY(TableScene)

    void mouseMoveEvent(QGraphicsSceneMouseEvent* event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event);
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event);
};

#endif // TABLE_SCENE_H
