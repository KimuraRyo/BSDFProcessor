// =================================================================== //
// Copyright (C) 2014-2019 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

/*!
 * \file    Utility.h
 * \brief   Utility functions.
 */

#ifndef UTILITY_H
#define UTILITY_H

#include <QColor>

#include <osg/Vec3>
#include <osg/Vec4>

#include <libbsdf/Common/Vector.h>

namespace util {

/*! Converts from osg::Vec3 to QColor. */
inline QColor osgToQt(const osg::Vec3& vec)
{
    return QColor(vec.x() * 255.0 + 0.5, vec.y() * 255.0 + 0.5, vec.z() * 255.0 + 0.5);
}

/*! Converts from osg::Vec4 to QColor. */
inline QColor osgToQt(const osg::Vec4& vec)
{
    return QColor(vec.x() * 255.0 + 0.5, vec.y() * 255.0 + 0.5, vec.z() * 255.0 + 0.5, vec.w() * 255.0 + 0.5);
}

/*! Converts from QColor to osg::Vec3. */
inline osg::Vec3 qtToOsg(const QColor& color)
{
    return osg::Vec3(color.red() / 255.0, color.green() / 255.0, color.blue() / 255.0);
}

/*! Converts from QColor to lb::Vec3. */
inline lb::Vec3 qtToLb(const QColor& color)
{
    qreal r, g, b;
    color.getRgbF(&r, &g, &b);
    return lb::Vec3(r, g, b);
}

/*! Converts from lb::Vec3 to QColor. */
inline QColor lbToQt(const lb::Vec3& color)
{
    return QColor::fromRgbF(color[0], color[1], color[2]);
}

} // namespace util

#endif // UTILITY_H
