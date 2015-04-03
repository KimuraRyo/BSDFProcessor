// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef SPECULAR_CENTERED_COORDINATE_SYSTEM_H
#define SPECULAR_CENTERED_COORDINATE_SYSTEM_H

#include <Eigen/Geometry>

#include <libbsdf/Common/SphericalCoordinateSystem.h>

/*!
 * \class   SpecularCenteredCoordinateSystem
 * \brief   The SpecularCenteredCoordinateSystem class provides the functions of a specular-centered coordinate system.
 *
 * The coordinate system has four angle parameters.
 *   - inTheta: the polar angle of an incoming direction
 *   - inPhi: the azimuthal angle of an incoming direction
 *   - specTheta: the polar angle of a specular direction
 *   - specPhi: the azimuthal angle of a specular direction
 *
 * Spec is an abbreviation for specular.
 */
class SpecularCenteredCoordinateSystem
{
public:
    /*!
     * Converts from four angles to incoming and outgoing directions and
     * assigns them to *inDir and *outDir.
     */
    static inline void toXyz(float inTheta, float inPhi, float specTheta, float specPhi,
                             lb::Vec3* inDir, lb::Vec3* outDir)
    {
        *inDir = lb::SphericalCoordinateSystem::toXyz(inTheta, inPhi);
        *outDir = toOutDirXyz(inTheta, specTheta, specPhi);
    }

    static const std::string ANGLE0_NAME; /*!< This attribute holds the name of inTheta. */
    static const std::string ANGLE1_NAME; /*!< This attribute holds the name of inPhi. */
    static const std::string ANGLE2_NAME; /*!< This attribute holds the name of specTheta. */
    static const std::string ANGLE3_NAME; /*!< This attribute holds the name of specPhi. */

    static const float MAX_ANGLE0; /*!< This attribute holds the maximum value of inTheta. */
    static const float MAX_ANGLE1; /*!< This attribute holds the maximum value of inPhi. */
    static const float MAX_ANGLE2; /*!< This attribute holds the maximum value of specTheta. */
    static const float MAX_ANGLE3; /*!< This attribute holds the maximum value of specPhi. */

private:
    /*! Converts an outgoing direction from a specular-centered coordinate system to a Cartesian. */
    static inline lb::Vec3 toOutDirXyz(float inTheta, float specTheta, float specPhi)
    {
        lb::Vec3 xyzVec = lb::SphericalCoordinateSystem::toXyz(specTheta, specPhi);
        float rotAngle = inTheta * (1.0f - specTheta / MAX_ANGLE2);
        lb::Vec2f rotVec = Eigen::Rotation2D<lb::Vec2f::Scalar>(rotAngle) * lb::Vec2f(xyzVec(0), xyzVec(2));

        return lb::Vec3(rotVec(0), xyzVec(1), rotVec(1));
    }
};

#endif // SPECULAR_CENTERED_COORDINATE_SYSTEM_H
