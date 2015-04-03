// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

/*!
 * \file    SceneUtil.h
 * \brief   Utility functions for the scene data.
 */

#ifndef SCENE_UTIL_H
#define SCENE_UTIL_H

#include <cmath>

#include <osg/Camera>
#include <osgText/Text>
#include <osgViewer/Viewer>

#include <QtGui/QColor>

#include <libbsdf/Brdf/Brdf.h>
#include <libbsdf/Common/SpecularCoordinateSystem.h>
#include <libbsdf/Common/SphericalCoordinateSystem.h>

#include "SpecularCenteredCoordinateSystem.h"

/*! Node mask types. */
namespace NodeMask {

enum Type {
    HUD = 1 << 0,
    BRDF = 1 << 1,
    SPECULAR_REFLECTANCE = 1 << 2,
    UNDEFINED = 1 << 3
};

} // namespace NodeMask

namespace scene_util {

template <typename T>
inline osg::Vec3 toOsg(const T& vec) { return osg::Vec3(vec[0], vec[1], vec[2]); }

/*! Corrects a gamma value. */
inline osg::Vec3d correctGamma(const osg::Vec3d& color, double gamma)
{
    return osg::Vec3d(std::pow(color.x(), 1.0 / gamma),
                      std::pow(color.y(), 1.0 / gamma),
                      std::pow(color.z(), 1.0 / gamma));
}

/*! Corrects a gamma value. */
inline osg::Vec4d correctGamma(const osg::Vec4d& color, double gamma)
{
    return osg::Vec4d(std::pow(color.x(), 1.0 / gamma),
                      std::pow(color.y(), 1.0 / gamma),
                      std::pow(color.z(), 1.0 / gamma), color.w());
}

/*! Calculates RGB using hue rotation. */
inline osg::Vec3 hueToRgb(float hue)
{
    // blue -> green -> yellow -> red
    double h = lb::clamp((1.0 - hue) * 240.0 / 360.0, 0.0, 1.0);
    QColor rgb = QColor::fromHsvF(h, 1.0, 1.0);
    return osg::Vec3(rgb.redF(), rgb.greenF(), rgb.blueF());
}

/*!
 * Fits the camera position to render the entire scene. The camera looks at the origin.
 *
 * \param camera            Camera to draw a view.
 * \param cameraDirection   Direction from the origin to a camera.
 * \param upDirection       Upward direction of a camera.
 * \param node              Scene node. If this is 0, this function uses the scene of camera.
 */
void fitCameraPosition(osg::Camera*     camera,
                       const osg::Vec3& cameraDirection,
                       const osg::Vec3& upDirection,
                       osg::Node*       node = 0);

/*! Computes the center of a node. */
osg::Vec3 computeCenter(osg::Node* node);

/*!
 * Creates a post processing group.
 *
 * \param subgraph              Rendered scene.
 * \param width                 Width of a rendered image.
 * \param height                Height of a rendered image.
 * \param useAlpha              Use alpha channel of texture.
 * \param useHdr                Use floating-point format of texture.
 * \param fragmentShader        Fragment shader for post processing.
 * \param fboWidth              Width of FBO texture. If this is 0, this parameter is processed the same as width.
 * \param fboHeight             Height of FBO texture. If this is 0, this parameter is processed the same as height.
 * \param numFboSamples         The number of FBO samples.
 * \param numFboColorSamples    The number of FBO color samples.
 * \return                      Scene group for post processing.
 */
osg::Group* createPostProcessingGroup(osg::Node*    subgraph,
                                      int           width,
                                      int           height,
                                      bool          useAlpha = false,
                                      bool          useHdr = true,
                                      osg::Shader*  fragmentShader = 0,
                                      int           fboWidth = 0,
                                      int           fboHeight = 0,
                                      int           numFboSamples = 0,
                                      int           numFboColorSamples = 0);

/*! Creates a group of order independent transparency. */
osg::Group* createOitGroup(osg::Group*  subgraph,
                           int          width,
                           int          height,
                           int          numPasses,
                           int          numFboSamples = 0,
                           int          numFboColorSamples = 0,
                           int          prevDepthTextureUnit = 0,
                           int          peelFboRenderOrder = 0);

/*! Creates an HUD camera displaying a text. */
osg::Camera* createHudText(const std::string& textString, int width, int height);

/*! Creates a text label. */
osgText::Text* createTextLabel(const std::string&           textString,
                               const osg::Vec3&             position,
                               osgText::Text::AlignmentType alignment = osgText::Text::BASE_LINE,
                               const osg::Vec4&             color = osg::Vec4(1.0, 1.0, 1.0, 1.0),
                               bool                         useClusterCulling = false,
                               const osg::Vec3&             normal = osg::Vec3(0.0, 0.0, 1.0),
                               float                        deviation = 0.0f);

/*! Creates a text label. */
osgText::Text* createTextLabel(const std::string&           textString,
                               const osg::Vec3&             position,
                               osgText::Font*               font,
                               osgText::Text::AlignmentType alignment = osgText::Text::BASE_LINE,
                               const osg::Vec4&             color = osg::Vec4(1.0, 1.0, 1.0, 1.0),
                               bool                         useClusterCulling = false,
                               const osg::Vec3&             normal = osg::Vec3(0.0, 0.0, 1.0),
                               float                        deviation = 0.0f);

/*! Picks a node and position. */
osg::Node* pickNode(osgViewer::View*    view,
                    const osg::Vec2&    cursorPosition,
                    osg::Vec3&          worldIntersectPosition,
                    osg::Node::NodeMask traversalMask = 0xffffffff,
                    bool                useWindowCoordinates = true);

/*! Displays a node path. */
void displayNodePath(const osg::NodePath& nodePath);

/*! Displays OpenGL information. */
void displayGlInformation(osg::GraphicsContext* graphicsContext);

/*! Computes an orthogonal vector. */
osg::Vec3d computeOrthogonalVector(const osg::Vec3d& vector);

/*! Creates the mesh geometry of a BRDF. */
template <typename CoordSysT>
osg::Geometry* createBrdfMeshGeometry(const lb::Brdf&   brdf,
                                      float             inTheta,
                                      float             inPhi,
                                      int               spectrumIndex,
                                      bool              useLogPlot,
                                      float             baseOfLogarithm,
                                      bool              isBtdf,
                                      int               numTheta = 361,
                                      int               numPhi = 361);

/*! Creates the point geometry of a BRDF. */
osg::Geometry* createBrdfPointGeometry(const lb::Brdf&  brdf,
                                       int              inThetaIndex,
                                       int              inPhiIndex,
                                       int              spectrumIndex,
                                       bool             useLogPlot,
                                       float            baseOfLogarithm,
                                       bool             isBtdf);

/*! Creates the text labes of sample points. */
void attachBrdfTextLabels(osg::Geode*       geode,
                          const lb::Brdf&   brdf,
                          int               inThetaIndex,
                          int               inPhiIndex,
                          int               spectrumIndex,
                          bool              useLogPlot,
                          float             baseOfLogarithm,
                          bool              isBtdf);

/*! Creates the lines of XYZ axis. */
osg::Geode* createAxis(double length = 1.0, bool useRgb = false);

osg::Geometry* createCircleFloor(float  radius,
                                 int    numPoints,
                                 float  lineWidth = 1.0f,
                                 bool   useStipple = false,
                                 bool   useLogPlot = false,
                                 float  baseOfLogarithm = 10.0f);

osg::Geometry* createInDirLine();

} // namespace scene_util

#endif // SCENE_UTIL_H
