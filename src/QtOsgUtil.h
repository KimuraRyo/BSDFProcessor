// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

/*!
 * \file    QtOsgUtil.h
 * \brief   Utility functions for Qt and OpenSceneGraph.
 */

#ifndef QTOSG_UTIL_H
#define QTOSG_UTIL_H

#include <QtWidgets/QColorDialog>
#include <QtWidgets/QPushButton>

#include <osg/Vec3>
#include <osg/Vec4>

namespace qt_osg_util {

/*! Converts from osg::Vec3 to QColor. */
inline QColor getQColor(const osg::Vec3& vec)
{
    return QColor(vec.x() * 255.0 + 0.5, vec.y() * 255.0 + 0.5, vec.z() * 255.0 + 0.5);
}

/*! Converts from osg::Vec4 to QColor. */
inline QColor getQColor(const osg::Vec4& vec)
{
    return QColor(vec.x() * 255.0 + 0.5, vec.y() * 255.0 + 0.5, vec.z() * 255.0 + 0.5, vec.w() * 255.0 + 0.5);
}

/*! Converts from QColor to osg::Vec3. */
inline osg::Vec3 getOsgVec3(const QColor& color)
{
    return osg::Vec3(color.red() / 255.0, color.green() / 255.0, color.blue() / 255.0);
}

/*!
 * Gets the background color of QPushButton in a style sheet.
 * If a color isn't found, QColor::isValid() is false.
 */
QColor getBackgroundColor(QPushButton* button);

/*! Sets the background color of QPushButton. */
void setBackgroundColor(QPushButton* button, const QColor& color);

/*!
 * Sets the background color of QPushButton with a color dialog.
 * If a color isn't set, QColor::isValid() is false.
 */
QColor setBackgroundColor(QPushButton* button);

} // namespace qt_osg_util

#endif // QTOSG_UTIL_H
