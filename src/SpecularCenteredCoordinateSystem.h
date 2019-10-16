// =================================================================== //
// Copyright (C) 2014-2019 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef SPECULAR_CENTERED_COORDINATE_SYSTEM_H
#define SPECULAR_CENTERED_COORDINATE_SYSTEM_H

#include <libbsdf/Common/SphericalCoordinateSystem.h>

/*!
 * \struct  SpecularCenteredCoordinateSystem
 * \brief   The SpecularCenteredCoordinateSystem struct provides the functions of a specular-centered coordinate system.
 *
 * The coordinate system has four angle parameters.
 *   - \a inTheta: the polar angle of an incoming direction
 *   - \a inPhi: the azimuthal angle of an incoming direction
 *   - \a specTheta: the polar angle of a specular direction
 *   - \a specPhi: the azimuthal angle of a specular direction
 *
 * \a spec is an abbreviation for specular. \a inPhi is not used for isotropic BRDFs.
 */
struct SpecularCenteredCoordinateSystem
{
    /*!
     * Converts from four angles to incoming and outgoing directions and
     * assigns them to \a inDir and \a outDir.
     */
    template <typename ScalarT>
    static void toXyz(ScalarT   inTheta,
                      ScalarT   inPhi,
                      ScalarT   specTheta,
                      ScalarT   specPhi,
                      lb::Vec3* inDir,
                      lb::Vec3* outDir)
    {
        *inDir = lb::SphericalCoordinateSystem::toXyz(inTheta, inPhi);
        *outDir = toOutDirXyz(inTheta, inPhi, specTheta, specPhi);
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
    template <typename ScalarT>
    static lb::Vec3 toOutDirXyz(ScalarT inTheta,
                                ScalarT inPhi,
                                ScalarT specTheta,
                                ScalarT specPhi)
    {
        lb::Vec3 xyzVec = lb::SphericalCoordinateSystem::toXyz(specTheta, specPhi);
        lb::Vec2::Scalar rotAngle = inTheta * (1 - specTheta / MAX_ANGLE2);
        lb::Vec2 rotThVec = Eigen::Rotation2D<lb::Vec2::Scalar>(rotAngle) * lb::Vec2(xyzVec[0], xyzVec[2]);
        lb::Vec2 rotPhVec = Eigen::Rotation2D<lb::Vec2::Scalar>(inPhi) * lb::Vec2(rotThVec[0], xyzVec[1]);

        lb::Vec3 rotDir(static_cast<lb::Vec3::Scalar>(rotPhVec[0]),
                        static_cast<lb::Vec3::Scalar>(rotPhVec[1]),
                        static_cast<lb::Vec3::Scalar>(rotThVec[1]));

        return rotDir.normalized();
    }

    /*! Converts an outgoing direction from a specular-centered coordinate system to a Cartesian. */
    template <typename ScalarT>
    static lb::Vec3 toOutDirXyz(ScalarT inTheta,
                                ScalarT specTheta,
                                ScalarT specPhi)
    {
        lb::Vec3 xyzVec = lb::SphericalCoordinateSystem::toXyz(specTheta, specPhi);
        lb::Vec2::Scalar rotAngle = inTheta * (1 - specTheta / MAX_ANGLE2);
        lb::Vec2 rotVec = Eigen::Rotation2D<lb::Vec2::Scalar>(rotAngle) * lb::Vec2(xyzVec[0], xyzVec[2]);

        lb::Vec3 rotDir(static_cast<lb::Vec3::Scalar>(rotVec[0]),
                        static_cast<lb::Vec3::Scalar>(xyzVec[1]),
                        static_cast<lb::Vec3::Scalar>(rotVec[1]));

        return rotDir.normalized();
    }
};

#endif // SPECULAR_CENTERED_COORDINATE_SYSTEM_H
