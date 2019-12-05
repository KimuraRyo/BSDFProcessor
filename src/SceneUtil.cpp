// =================================================================== //
// Copyright (C) 2014-2019 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "SceneUtil.h"

#include <QColor>
#include <QString>

#include <osg/BlendEquation>
#include <osg/BlendFunc>
#include <osg/ClipPlane>
#include <osg/ClusterCullingCallback>
#include <osg/ComputeBoundsVisitor>
#include <osg/Depth>
#include <osg/LineStipple>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/PolygonOffset>
#include <osg/PolygonMode>
#include <osg/Texture2D>
#include <osg/io_utils>
#include <osgDB/WriteFile>
#include <osgText/FadeText>
#include <osgText/Font>
#include <osgUtil/SmoothingVisitor>

#include <libbsdf/Brdf/HalfDifferenceCoordinatesBrdf.h>
#include <libbsdf/Brdf/SpecularCoordinatesBrdf.h>
#include <libbsdf/Common/SpectrumUtility.h>

#include "SpecularCenteredCoordinateSystem.h"

float scene_util::spectrumToY(const lb::Spectrum&   spectrum,
                              lb::ColorModel        colorModel,
                              const lb::Arrayf&     wavelengths)
{
    if (colorModel == lb::RGB_MODEL) {
        lb::Vec3f xyz = lb::srgbToXyz<lb::Vec3f>(spectrum);
        return xyz[1];
    }
    else if (colorModel == lb::SPECTRAL_MODEL) {
        return lb::SpectrumUtility::spectrumToY(spectrum, wavelengths);
    }
    else {
        lbError << "[scene_util::spectrumToY] Invalid color model for photometric values: " << colorModel;
        return 0.0f;
    }
}

void scene_util::fitCameraPosition(osg::Camera*     camera,
                                   const osg::Vec3& cameraDirection,
                                   const osg::Vec3& upDirection,
                                   osg::Node*       node)
{
    using std::min;
    using std::abs;

    osg::ComputeBoundsVisitor cbv(osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN);
    if (node) {
        node->accept(cbv);
    }
    else {
        camera->asGroup()->osg::Group::traverse(cbv);
    }

    osg::BoundingBox& bb = cbv.getBoundingBox();
    if (!bb.valid()) return;

    double distance;

    const double minThreshold = 0.01;
    double left, right, bottom, top, zNear, zFar;
    if (bb.radius() < minThreshold) {
        distance = minThreshold;
    }
    else if (camera->getProjectionMatrixAsFrustum(left, right, bottom, top, zNear, zFar)) {
        double minNearPlaneComponent = min(min(abs(left), abs(right)), min(abs(bottom), abs(top)));
        double minFovRadian = std::atan2(minNearPlaneComponent, zNear);

        osg::BoundingBox expandedBb(bb);
        expandedBb.expandBy(-bb._min);
        expandedBb.expandBy(-bb._max);
        osg::BoundingSphere bs(expandedBb);

        distance = bs.radius() / std::sin(minFovRadian);
    }
    else if (camera->getProjectionMatrixAsOrtho(left, right, bottom, top, zNear, zFar)) {
        distance = abs(zFar - zNear) / 2.0;
    }
    else {
        distance = 3.5 * bb.radius();
    }

    osg::Vec3 viewDir(cameraDirection);
    viewDir.normalize();
    viewDir *= distance;
    camera->setViewMatrixAsLookAt(viewDir, osg::Vec3d(0.0, 0.0, 0.0), upDirection);
}

osg::Vec3 scene_util::computeCenter(osg::Node* node)
{
    osg::ComputeBoundsVisitor cbv(osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN);
    node->accept(cbv);
    return cbv.getBoundingBox().center();
}

osg::Group* scene_util::createPostProcessingGroup(osg::Node*    subgraph,
                                                  int           width,
                                                  int           height,
                                                  bool          useAlpha,
                                                  bool          useHdr,
                                                  osg::Shader*  fragmentShader,
                                                  int           fboWidth,
                                                  int           fboHeight,
                                                  int           numFboSamples,
                                                  int           numFboColorSamples)
{
    if (width <= 0 || height <= 0) return 0;

    osg::ref_ptr<osg::Group> postProcessingGroup = new osg::Group;
    postProcessingGroup->setName("postProcessingGroup");

    if (fboWidth <= 0) {
        fboWidth = width;
    }

    if (fboHeight <= 0) {
        fboHeight = height;
    }

    bool mipmapUsed = false;
    if (width != fboWidth || height != fboHeight) {
        mipmapUsed = true;
    }

    // Set up an FBO texture.
    osg::Texture2D* fboTexture = new osg::Texture2D;
    {
        fboTexture->setTextureSize(fboWidth, fboHeight);

        if (mipmapUsed) {
            fboTexture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR_MIPMAP_LINEAR);
            fboTexture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
        }
        else {
            fboTexture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
            fboTexture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
        }

        if (useAlpha) {
            if (useHdr) {
                fboTexture->setInternalFormat(GL_RGBA32F_ARB);
                fboTexture->setSourceFormat(GL_RGBA);
                fboTexture->setSourceType(GL_FLOAT);
            }
            else {
                fboTexture->setInternalFormat(GL_RGBA);
                fboTexture->setSourceFormat(GL_RGBA);
                fboTexture->setSourceType(GL_UNSIGNED_BYTE);
            }
        }
        else
        {
            if (useHdr) {
                fboTexture->setInternalFormat(GL_RGB32F_ARB);
                fboTexture->setSourceFormat(GL_RGB);
                fboTexture->setSourceType(GL_FLOAT);
            }
            else {
                fboTexture->setInternalFormat(GL_RGB);
                fboTexture->setSourceFormat(GL_RGB);
                fboTexture->setSourceType(GL_UNSIGNED_BYTE);
            }
        }
    }

    // Set up an FBO camera.
    {
        osg::Camera* fboCamera = new osg::Camera;
        fboCamera->setName("fboCamera");
        fboCamera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        fboCamera->setReferenceFrame(osg::Transform::RELATIVE_RF);
        fboCamera->setProjectionMatrix(osg::Matrixd::identity());
        fboCamera->setViewMatrix(osg::Matrixd::identity());
        fboCamera->setViewport(0, 0, fboWidth, fboHeight);
        fboCamera->setRenderOrder(osg::Camera::PRE_RENDER);
        fboCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
        fboCamera->attach(osg::Camera::COLOR_BUFFER, fboTexture, 0, 0, mipmapUsed, numFboSamples, numFboColorSamples);

        postProcessingGroup->addChild(fboCamera);
        fboCamera->addChild(subgraph);
    }

    // Set up an HUD camera.
    {
        int hudWidth = width;
        int hudHeight = height;
        osg::Geode* hudGeode = new osg::Geode;
        hudGeode->setName("hudGeode");
        hudGeode->setNodeMask(HUD_MASK);

        const double hudDepth = 0.0;
        osg::Geometry* hudGeom = osg::createTexturedQuadGeometry(osg::Vec3(0.0,         0.0,        hudDepth),
                                                                 osg::Vec3(hudWidth,    0.0,        hudDepth),
                                                                 osg::Vec3(0.0,         hudHeight,  hudDepth));
        hudGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        hudGeom->getOrCreateStateSet()->setTextureAttributeAndModes(0, fboTexture, osg::StateAttribute::ON);
        hudGeode->addDrawable(hudGeom);

        osg::Group* ppsGroup = new osg::Group;
        ppsGroup->setName("ppsGroup");
        ppsGroup->addChild(hudGeode);
        if (fragmentShader) {
            osg::Program* program = new osg::Program;
            program->addShader(fragmentShader);
            ppsGroup->getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON);
            ppsGroup->getOrCreateStateSet()->setName("postProcessingShader");
        }

        osg::Camera* hudCamera = new osg::Camera;
        hudCamera->setName("hudCamera");
        hudCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
        hudCamera->setViewMatrix(osg::Matrix::identity());
        hudCamera->setProjectionMatrixAsOrtho2D(0, hudWidth, 0, hudHeight);
        hudCamera->setRenderOrder(osg::Camera::NESTED_RENDER);

        postProcessingGroup->addChild(hudCamera);
        hudCamera->addChild(ppsGroup);
    }

    return postProcessingGroup.release();
}

osg::Group* scene_util::createOitGroup(osg::Group*  subgraph,
                                       int          width,
                                       int          height,
                                       int          numPasses,
                                       int          numFboSamples,
                                       int          numFboColorSamples,
                                       int          prevDepthTextureUnit,
                                       int          peelFboRenderOrder)
{
    if (width <= 0 || height <= 0 || numPasses <= 0) return 0;

    osg::ref_ptr<osg::Group> oitGroup = new osg::Group;
    oitGroup->setName("oitGroup");

    osg::Uniform* viewSizeUniform = new osg::Uniform(osg::Uniform::INT_VEC2, "viewSize");
    viewSizeUniform->set(width, height);
    subgraph->getOrCreateStateSet()->addUniform(viewSizeUniform);

    osg::Texture2D* depthTextures[2];
    for (int i = 0; i < 2; ++i) {
        depthTextures[i] = new osg::Texture2D;
        depthTextures[i]->setTextureSize(width, height);
        depthTextures[i]->setInternalFormat(GL_DEPTH_COMPONENT24);
        depthTextures[i]->setSourceFormat(GL_DEPTH_COMPONENT);
        depthTextures[i]->setSourceType(GL_FLOAT);
        depthTextures[i]->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
        depthTextures[i]->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
    }

    // Set up a camera for blending.
    osg::Camera* hudCamera = new osg::Camera;
    {
        hudCamera->setName("hudCamera");
        hudCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
        hudCamera->setViewMatrix(osg::Matrix::identity());
        hudCamera->setProjectionMatrixAsOrtho2D(0, width, 0, height);
        hudCamera->setRenderOrder(osg::Camera::NESTED_RENDER);
        oitGroup->addChild(hudCamera);

        osg::BlendEquation* blendEquation = new osg::BlendEquation(osg::BlendEquation::FUNC_ADD);
        osg::StateSet* stateSet = hudCamera->getOrCreateStateSet();
        stateSet->setAttributeAndModes(blendEquation, osg::StateAttribute::ON);
        stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    }

    for (int i = 0; i < numPasses; ++i) {
        int currId = i % 2;
        int prevId = 1 - currId;

        osg::Texture2D* colorTexture = new osg::Texture2D;
        colorTexture->setTextureSize(width, height);
        colorTexture->setInternalFormat(GL_RGBA32F_ARB);
        colorTexture->setSourceFormat(GL_RGBA);
        colorTexture->setSourceType(GL_FLOAT);
        colorTexture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
        colorTexture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);

        // Peeling
        {
            osg::Camera* peelFboCamera = new osg::Camera;
            peelFboCamera->setName("peelFboCamera");
            peelFboCamera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            peelFboCamera->setClearColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));
            peelFboCamera->setReferenceFrame(osg::Transform::RELATIVE_RF);
            peelFboCamera->setProjectionMatrix(osg::Matrixd::identity());
            peelFboCamera->setViewMatrix(osg::Matrixd::identity());
            peelFboCamera->setViewport(0, 0, width, height);
            peelFboCamera->setRenderOrder(osg::Camera::PRE_RENDER, peelFboRenderOrder + i);
            peelFboCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

            // Use multiple render targets.
            peelFboCamera->attach(osg::Camera::COLOR_BUFFER, colorTexture,
                                  0, 0, false, numFboSamples, numFboColorSamples);
            peelFboCamera->attach(osg::Camera::DEPTH_BUFFER, depthTextures[currId],
                                  0, 0, false, numFboSamples, numFboColorSamples);

            osg::StateSet* stateSet = peelFboCamera->getOrCreateStateSet();

            osg::Uniform* firstPassUniform = new osg::Uniform(osg::Uniform::BOOL, "oitFirstPass");
            stateSet->addUniform(firstPassUniform);

            if (i == 0) {
                firstPassUniform->set(true);
            }
            else {
                firstPassUniform->set(false);

                stateSet->setTextureAttributeAndModes(prevDepthTextureUnit, depthTextures[prevId]);
                osg::Uniform* prevDepthTexUniform = new osg::Uniform("oitPrevDepthTexture", prevDepthTextureUnit);
                stateSet->addUniform(prevDepthTexUniform);
            }

            osg::Group* peelScene = new osg::Group;
            peelScene->setName("peelScene");
            peelScene->addChild(subgraph);

            oitGroup->addChild(peelFboCamera);
            peelFboCamera->addChild(peelScene);
        }

        // Blending
        {
            osg::Geode* hudGeode = new osg::Geode;
            hudGeode->setName("hudGeode");
            hudGeode->setNodeMask(HUD_MASK);
            hudCamera->addChild(hudGeode);

            float depth = -i;
            osg::Geometry* hudGeom = osg::createTexturedQuadGeometry(osg::Vec3(0.0,   0.0,    depth),
                                                                     osg::Vec3(width, 0.0,    depth),
                                                                     osg::Vec3(0.0,   height, depth));

            osg::StateSet* stateSet = hudGeom->getOrCreateStateSet();
            stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
            stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
            stateSet->setAttribute(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON);
            stateSet->setTextureAttributeAndModes(0, colorTexture);

            hudGeode->addDrawable(hudGeom);
        }
    }

    return oitGroup.release();
}

osg::Camera* scene_util::createHudText(const std::string& textString, int width, int height)
{
    if (width <= 0 || height <= 0) return 0;

    osg::Geode* geode = new osg::Geode;
    geode->setName("hudTextGeode");
    geode->setNodeMask(HUD_MASK);

    osgText::Text* text = new osgText::Text;
    text->setFont(ARIAL_FONT_FILE);
    text->setCharacterSize(20.0f);
    text->setColor(osg::Vec4(1.0, 1.0, 1.0, 1.0));

    text->setPosition(osg::Vec3(10.0, 10.0, 0.0));
    text->setLayout(osgText::Text::LEFT_TO_RIGHT);
    text->setText(textString);

    geode->addDrawable(text);

    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setProjectionMatrixAsOrtho2D(0, width, 0, height);
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    camera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    camera->setRenderOrder(osg::Camera::POST_RENDER, 1000);
    camera->setAllowEventFocus(false);
    camera->addChild(geode);

    return camera.release();
}

osgText::Text* scene_util::createTextLabel(const std::string&           textString,
                                           const osg::Vec3&             position,
                                           const float                  fadeSpeed,
                                           const std::string&           fontFileName,
                                           osgText::Text::AlignmentType alignment,
                                           const osg::Vec4&             color,
                                           bool                         useClusterCulling,
                                           const osg::Vec3&             normal,
                                           float                        deviation)
{
    osg::ref_ptr<osgText::FadeText> text = new osgText::FadeText;
    text->setFadeSpeed(fadeSpeed);

    text->setText(textString);
    text->setPosition(position);
    text->setAlignment(alignment);
    text->setColor(color);
    text->setFont(fontFileName);

#if OSG_VERSION_GREATER_OR_EQUAL(3, 6, 0)
    text->setCharacterSize(18.0f);
    text->setFontResolution(32, 32);
#else
    text->setCharacterSize(28.0f);
    text->setFontResolution(28, 28);
#endif

    text->setAxisAlignment(osgText::Text::SCREEN);
    text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
    text->setAutoRotateToScreen(true);

    if (useClusterCulling) {
        text->setCullCallback(new osg::ClusterCullingCallback(position, normal, deviation));
    }

    return text.release();
}

osg::Node* scene_util::pickNode(osgViewer::View*    view,
                                const osg::Vec2&    cursorPosition,
                                osg::Vec3&          worldIntersectPosition,
                                osg::Node::NodeMask traversalMask,
                                bool                useWindowCoordinates)
{
    if (!view->getScene()) return 0;

    osgUtil::PolytopeIntersector* intersector;
    if (useWindowCoordinates) {
        // Use window coordinates.
        constexpr double w = 0.5;
        constexpr double h = 0.5;
        intersector = new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW,
                                                       cursorPosition.x() - w,
                                                       cursorPosition.y() - h,
                                                       cursorPosition.x() + w,
                                                       cursorPosition.y() + h);
    }
    else {
        // Use projection/clip space.
        osg::Viewport* viewport = view->getCamera()->getViewport();
        double x = cursorPosition.x() / viewport->width()  * 2.0 - 1.0;
        double y = cursorPosition.y() / viewport->height() * 2.0 - 1.0;

        constexpr double w = 0.005;
        constexpr double h = 0.005;
        intersector = new osgUtil::PolytopeIntersector(osgUtil::Intersector::PROJECTION,
                                                       x - w,
                                                       y - h,
                                                       x + w,
                                                       y + h);
    }

    // Remove the near plane of a polytope, because it is a different distance from the near plane of frustum in some cases.
    auto& planes = intersector->getPolytope().getPlaneList();
    planes.erase(planes.begin() + 4);

    osgUtil::IntersectionVisitor iv(intersector);
    iv.setTraversalMask(traversalMask);
    view->getCamera()->accept(iv);

    if (intersector->containsIntersections()) {
        for (auto intersection : intersector->getIntersections()) {
            if (!intersection.nodePath.empty() &&
                intersection.distance > -1.0) {
                lbDebug
                    << "[scene_util::pickNode]"
                    << "\n\tpicked node: "                      << intersection.nodePath.back()->getName()
                    << "\n\tposition: "                         << intersection.localIntersectionPoint
                    << "\n\tnode type: "                        << intersection.nodePath.back()->className()
                    << "\n\tdistance from reference plane: "    << intersection.distance
                    << "\n\tmax distance: "                     << intersection.maxDistance
                    << "\n\tprimitive index: "                  << intersection.primitiveIndex
                    << "\n\tnumIntersectionPoints: "            << intersection.numIntersectionPoints;

                auto geode = dynamic_cast<osg::Geode*>(intersection.nodePath.back());
                if (geode) {
                    if (intersection.matrix.valid()) {
                        worldIntersectPosition = intersection.localIntersectionPoint * (*intersection.matrix);
                    }
                    else {
                        worldIntersectPosition = intersection.localIntersectionPoint;
                    }
                    return intersection.nodePath.back();
                }
            }
        }
    }

    return 0;
}

void scene_util::displayNodePath(const osg::NodePath& nodePath)
{
    std::string path("[scene_util::displayNodePath] ");

    for(osg::NodePath::const_iterator it = nodePath.begin(); it != nodePath.end(); ++it) {
        path += "/" + (*it)->getName();
    }

    lbTrace << path;
}

void scene_util::displayGlInformation(osg::GraphicsContext* graphicsContext)
{
    graphicsContext->makeCurrent();

    lbInfo << "GL_VENDOR: "                   << std::string((char*)glGetString(GL_VENDOR));
    lbInfo << "GL_RENDERER: "                 << std::string((char*)glGetString(GL_RENDERER));
    lbInfo << "GL_VERSION: "                  << std::string((char*)glGetString(GL_VERSION));
    lbInfo << "GL_SHADING_LANGUAGE_VERSION: " << std::string((char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

    graphicsContext->releaseContext();
}

osg::Vec3d scene_util::computeOrthogonalVector(const osg::Vec3d& vector)
{
    double length = vector.length();
    osg::Vec3d orthogonalVector = vector ^ osg::Vec3d(0.0, 1.0, 0.0);
    if (orthogonalVector.normalize() < length * 0.5) {
        orthogonalVector = vector ^ osg::Vec3d(0.0, 0.0, 1.0);
        orthogonalVector.normalize();
    }

    return orthogonalVector;
}

template <typename CoordSysT>
osg::Geometry* scene_util::createBrdfMeshGeometry(const lb::Brdf&   brdf,
                                                  float             inTheta,
                                                  float             inPhi,
                                                  int               wavelengthIndex,
                                                  bool              logPlotUsed,
                                                  float             baseOfLogarithm,
                                                  lb::DataType      dataType,
                                                  bool              photometric,
                                                  int               numTheta,
                                                  int               numPhi)
{
    const bool hueUsed = false;

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setName("meshGeom");

    lb::Arrayf thetaAngles;
    if (dynamic_cast<const lb::SpecularCoordinatesBrdf*>(&brdf) ||
        dynamic_cast<const lb::HalfDifferenceCoordinatesBrdf*>(&brdf)) {
        // Create narrow intervals near specular directions.
        thetaAngles = lb::createExponentialArray<lb::Arrayf>(numTheta, CoordSysT::MAX_ANGLE2, 2.0f);
    }
    else {
        thetaAngles = lb::Arrayf::LinSpaced(numTheta, 0.0, CoordSysT::MAX_ANGLE2);
    }

    lb::Arrayf phiAngles = lb::Arrayf::LinSpaced(numPhi, 0.0, CoordSysT::MAX_ANGLE3);

    std::vector<lb::Vec3, Eigen::aligned_allocator<lb::Vec3>> positions;
    positions.reserve(numTheta * numPhi);

    // An incoming polar angle of zero is offset to validate an incoming azimuthal angle.
    float offsetInTheta = std::max(inTheta, lb::EPSILON_F);

    float maxBrdfValue = 0.0f;

    // Create outgoing directions.
    for (int phIndex = 0; phIndex < numPhi;   ++phIndex) {
    for (int thIndex = 0; thIndex < numTheta; ++thIndex) {
        lb::Vec3 inDir, outDir;
        CoordSysT::toXyz(offsetInTheta, inPhi, thetaAngles[thIndex], phiAngles[phIndex], &inDir, &outDir);

        if (outDir[2] < 0.0) {
            outDir[2] = 0.0;

            if (outDir == lb::Vec3::Zero()) {
                positions.push_back(lb::Vec3::Zero());
                continue;
            }
        }
        outDir.normalize();

        float brdfValue;
        if (photometric) {
            const lb::SampleSet* ss = brdf.getSampleSet();
            lb::Spectrum sp = brdf.getSpectrum(inDir, outDir);
            brdfValue = spectrumToY(sp, ss->getColorModel(), ss->getWavelengths());
        }
        else {
            brdfValue = brdf.getValue(inDir, outDir, wavelengthIndex);
        }

        if (brdfValue <= 0.0f) {
            positions.push_back(lb::Vec3::Zero());
            continue;
        }

        if (logPlotUsed) {
            brdfValue = toLogValue(brdfValue, baseOfLogarithm);
        }

        if (maxBrdfValue < brdfValue) {
            maxBrdfValue = brdfValue;
        }

        if (dataType == lb::BTDF_DATA) {
            outDir[2] = -outDir[2];
        }

        positions.push_back(outDir * brdfValue);
    }}

    osg::Vec3Array* vertices = new osg::Vec3Array;
    vertices->reserve((numTheta - 1) * (numPhi - 1) * 4);

    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->reserve((numTheta - 1) * (numPhi - 1) * 4);

    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->reserve((numTheta - 1) * (numPhi - 1) * 4);

    // Create vertices and normals.
    for (int phIndex = 0; phIndex < numPhi   - 1; ++phIndex) {
    for (int thIndex = 0; thIndex < numTheta - 1; ++thIndex) {
        int nextThIndex = thIndex + 1;
        int nextPhIndex = phIndex + 1;

        int i0 = thIndex     + numTheta * phIndex;
        int i1 = nextThIndex + numTheta * phIndex;
        int i2 = nextThIndex + numTheta * nextPhIndex;
        int i3 = thIndex     + numTheta * nextPhIndex;

        lb::Vec3 pos0 = positions.at(i0);
        lb::Vec3 pos1 = positions.at(i1);
        lb::Vec3 pos2 = positions.at(i2);
        lb::Vec3 pos3 = positions.at(i3);

        if (dataType == lb::BRDF_DATA) {
            if (pos0[2] <= 0.0 && pos1[2] <= 0.0 && pos2[2] <= 0.0 && pos3[2] <= 0.0) {
                continue;
            }
        }
        else if (dataType == lb::BTDF_DATA) {
            if (pos0[2] >= 0.0 && pos1[2] >= 0.0 && pos2[2] >= 0.0 && pos3[2] >= 0.0) {
                continue;
            }
        }

        vertices->push_back(toOsg(pos0));
        vertices->push_back(toOsg(pos1));
        vertices->push_back(toOsg(pos2));
        vertices->push_back(toOsg(pos3));

        lb::Vec3 normal = (pos2 - pos0).cross(pos3 - pos1);
        normal.normalize();
        normals->push_back(toOsg(normal));
        normals->push_back(toOsg(normal));
        normals->push_back(toOsg(normal));
        normals->push_back(toOsg(normal));

        if (hueUsed) {
            osg::Vec3 rgb0 = scene_util::hueToRgb(pos0.norm() / maxBrdfValue);
            osg::Vec3 rgb1 = scene_util::hueToRgb(pos1.norm() / maxBrdfValue);
            osg::Vec3 rgb2 = scene_util::hueToRgb(pos2.norm() / maxBrdfValue);
            osg::Vec3 rgb3 = scene_util::hueToRgb(pos3.norm() / maxBrdfValue);

            colors->push_back(osg::Vec4(rgb0, 1.0));
            colors->push_back(osg::Vec4(rgb1, 1.0));
            colors->push_back(osg::Vec4(rgb2, 1.0));
            colors->push_back(osg::Vec4(rgb3, 1.0));
        }
    }}

    geom->setVertexArray(vertices);
    geom->setNormalArray(normals, osg::Array::BIND_PER_VERTEX);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, vertices->size()));

    if (hueUsed) {
        geom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
    }
    else {
        colors->push_back(osg::Vec4(0.6, 0.6, 0.6, 1.0));
        geom->setColorArray(colors, osg::Array::BIND_OVERALL);
    }

    //osgUtil::SmoothingVisitor sv;
    //sv.smooth(*geom);

    return geom.release();
}

template osg::Geometry* scene_util::createBrdfMeshGeometry<lb::SphericalCoordinateSystem>(
    const lb::Brdf& brdf, float inTheta, float inPhi, int wavelengthIndex,
    bool logPlotUsed, float baseOfLogarithm, lb::DataType dataType, bool photometric,
    int numTheta, int numPhi);

template osg::Geometry* scene_util::createBrdfMeshGeometry<lb::SpecularCoordinateSystem>(
    const lb::Brdf& brdf, float inTheta, float inPhi, int wavelengthIndex,
    bool logPlotUsed, float baseOfLogarithm, lb::DataType dataType, bool photometric,
    int numTheta, int numPhi);

template osg::Geometry* scene_util::createBrdfMeshGeometry<SpecularCenteredCoordinateSystem>(
    const lb::Brdf& brdf, float inTheta, float inPhi, int wavelengthIndex,
    bool logPlotUsed, float baseOfLogarithm, lb::DataType dataType, bool photometric,
    int numTheta, int numPhi);

osg::Geometry* scene_util::createBrdfPointGeometry(const lb::Brdf&  brdf,
                                                   int              inThetaIndex,
                                                   int              inPhiIndex,
                                                   int              wavelengthIndex,
                                                   bool             logPlotUsed,
                                                   float            baseOfLogarithm,
                                                   lb::DataType     dataType)
{
    const lb::SampleSet* ss = brdf.getSampleSet();

    lb::Vec3 offsetInDir = lb::Vec3::Zero();

    // An incoming polar angle of zero is offset to validate an incoming azimuthal angle.
    if (inThetaIndex == 0 &&
        ss->getAngle0(inThetaIndex) == 0.0f) {
        float inPhi = ss->getAngle1(inPhiIndex);
        offsetInDir = lb::SphericalCoordinateSystem::toXyz(lb::EPSILON_F, inPhi);
    }

    osg::Vec3Array* vertices = new osg::Vec3Array;

    for (int i2 = 0; i2 < ss->getNumAngles2(); ++i2) {
    for (int i3 = 0; i3 < ss->getNumAngles3(); ++i3) {
        lb::Vec3 inDir, outDir;
        brdf.getInOutDirection(inThetaIndex, inPhiIndex, i2, i3, &inDir, &outDir);

        if (offsetInDir.z() != 0.0f) {
            inDir = offsetInDir;
        }

        if (lb::isDownwardDir(outDir)) continue;

        float brdfValue = brdf.getValue(inDir, outDir, wavelengthIndex);
        if (brdfValue <= 0.0f) continue;

        if (logPlotUsed) {
            brdfValue = toLogValue(brdfValue, baseOfLogarithm);
        }

        if (dataType == lb::BTDF_DATA) {
            outDir[2] = -outDir[2];
        }

        vertices->push_back(toOsg(outDir * brdfValue));
    }}

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    //geom->setName("pointGeom");

    osg::StateSet* stateSet = geom->getOrCreateStateSet();

    geom->setVertexArray(vertices);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));

    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(0.0f, 0.5f, 1.0f, 1.0f));
    geom->setColorArray(colors, osg::Array::BIND_OVERALL);

    //osg::PolygonMode *pm = new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::POINT);
    //stateSet->setAttributeAndModes(pm, osg::StateAttribute::ON);

    osg::Point* point = new osg::Point(1.0f);
    point->setMinSize(3.0f);
    point->setMaxSize(7.0f);
    point->setDistanceAttenuation(osg::Vec3(0.0f, 0.0f, 1.0f));
    stateSet->setAttribute(point);

    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    return geom.release();
}

void scene_util::attachBrdfTextLabels(osg::Geode*       geode,
                                      const lb::Brdf&   brdf,
                                      int               inThetaIndex,
                                      int               inPhiIndex,
                                      int               wavelengthIndex,
                                      bool              logPlotUsed,
                                      float             baseOfLogarithm,
                                      lb::DataType      dataType)
{
    const lb::SampleSet* ss = brdf.getSampleSet();

    const int maxSamples = 5000; // reduction threshold of samples

    osg::Vec3Array* pointVertices = new osg::Vec3Array;

    // Add labels.
    for (int i3 = 0; i3 < ss->getNumAngles3() - 1; ++i3) {
        // Avoid extra azimuthal angles.
        bool many = (ss->getNumAngles2() * (ss->getNumAngles3() - 1) > maxSamples);
        const float interval = lb::PI_2_F;
        float modulo = std::fmod(ss->getAngle3(i3), interval);
        bool onAxis = (modulo < lb::EPSILON_F * interval * 10.0f ||
                       modulo > interval - lb::EPSILON_F * interval * 10.0f);
        if (many && !onAxis) continue;

        for (int i2 = 0; i2 < ss->getNumAngles2(); ++i2) {
            if (i2 == 0 && i3 > 0) continue;

            lb::Vec3 inDir, outDir;
            brdf.getInOutDirection(inThetaIndex, inPhiIndex, i2, i3, &inDir, &outDir);

            if (lb::isDownwardDir(outDir)) {
                continue;
            }

            if (dataType == lb::BTDF_DATA) {
                outDir[2] = -outDir[2];
            }

            float brdfValue = ss->getSpectrum(inThetaIndex, inPhiIndex, i2, i3)[wavelengthIndex];
            if (brdfValue <= 0.0f) continue;

            float distance;
            if (logPlotUsed) {
                distance = toLogValue(brdfValue, baseOfLogarithm);
            }
            else {
                distance = brdfValue;
            }

            osg::Vec3 pos = toOsg(outDir * distance);
            pointVertices->push_back(pos);

            osgText::Text::AlignmentType alignment;
            if (dataType == lb::BTDF_DATA) {
                alignment = osgText::Text::LEFT_TOP;
            }
            else {
                alignment = osgText::Text::BASE_LINE;
            }
            std::string textString = QString::number(brdfValue).toLocal8Bit().data();
            osgText::Text* text = createTextLabel(textString + "   ", pos, 0.1f, ARIAL_FONT_FILE, alignment);
            text->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

            geode->addDrawable(text);
        }
    }

    // Add points.
    {
        osg::Geometry* pointGeom = new osg::Geometry;
        //pointGeom->setName("pointGeom");

        pointGeom->setVertexArray(pointVertices);
        pointGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, pointVertices->size()));

        osg::Vec4Array* pointColors = new osg::Vec4Array;
        pointColors->push_back(osg::Vec4(0.0f, 0.4f, 0.8f, 1.0f));
        pointGeom->setColorArray(pointColors, osg::Array::BIND_OVERALL);

        osg::Point* point = new osg::Point(1.0f);
        point->setMinSize(4.0f);
        point->setMaxSize(7.0f);
        point->setDistanceAttenuation(osg::Vec3(0.0f, 0.0f, 1.0f));
        pointGeom->getOrCreateStateSet()->setAttribute(point);

        geode->addDrawable(pointGeom);
    }

    osg::ClipPlane* clipPlane = new osg::ClipPlane;
    if (dataType == lb::BTDF_DATA) {
        clipPlane->setClipPlane(0.0, 0.0, -1.0, 0.000001);
    }
    else {
        clipPlane->setClipPlane(0.0, 0.0, 1.0, 0.000001);
    }

    // Add lines.
    for (int i3 = 0; i3 < ss->getNumAngles3() - 1; ++i3) {
        // Avoid extra azimuthal angles.
        bool many = (ss->getNumAngles2() * (ss->getNumAngles3() - 1) > maxSamples);
        const float interval = lb::PI_2_F;
        float modulo = std::fmod(ss->getAngle3(i3), interval);
        bool onAxis = (modulo < lb::EPSILON_F * interval * 10.0f ||
                       modulo > interval - lb::EPSILON_F * interval * 10.0f);
        if (many && !onAxis) continue;

        osg::Vec3Array* lineVertices = new osg::Vec3Array;
        bool zNegative = false;
        for (int i2 = 0; i2 < ss->getNumAngles2(); ++i2) {
            lb::Vec3 inDir, outDir;
            brdf.getInOutDirection(inThetaIndex, inPhiIndex, i2, i3, &inDir, &outDir);

            if (lb::isDownwardDir(outDir)) {
                if (zNegative) {
                    continue;
                }
                else {
                    zNegative = true;
                }
            }

            if (dataType == lb::BTDF_DATA) {
                outDir[2] = -outDir[2];
            }

            float brdfValue = ss->getSpectrum(inThetaIndex, inPhiIndex, i2, i3)[wavelengthIndex];
            float distance;
            if (logPlotUsed) {
                distance = toLogValue(brdfValue, baseOfLogarithm);
            }
            else {
                distance = brdfValue;
            }

            osg::Vec3 pos = toOsg(outDir * distance);
            lineVertices->push_back(pos);
        }

        osg::Geometry* lineGeom = new osg::Geometry;
        //lineGeom->setName("lineGeom");

        lineGeom->setVertexArray(lineVertices);
        lineGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, lineVertices->size()));

        osg::Vec4Array* lineColors = new osg::Vec4Array;
        lineColors->push_back(osg::Vec4(0.2, 0.2, 0.0, 1.0));
        lineGeom->setColorArray(lineColors, osg::Array::BIND_OVERALL);

        lineGeom->getOrCreateStateSet()->setAttributeAndModes(clipPlane, osg::StateAttribute::ON);

        geode->addDrawable(lineGeom);
    }

    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    //geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
}

osg::Geode* scene_util::createAxis(double length, bool backSideShown, bool rgbUsed)
{
    osg::ref_ptr<osg::Geode> axisGeode = new osg::Geode;
    axisGeode->setName("axisGeode");

    osg::Geometry* geom = new osg::Geometry;
    axisGeode->addDrawable(geom);

    osg::Vec3Array* vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(-length, 0.0, 0.0));
    vertices->push_back(osg::Vec3( length, 0.0, 0.0));
    vertices->push_back(osg::Vec3(0.0, -length, 0.0));
    vertices->push_back(osg::Vec3(0.0,  length, 0.0));
    if (backSideShown) {
        vertices->push_back(osg::Vec3(0.0, 0.0, -length));
    }
    else {
        vertices->push_back(osg::Vec3(0.0, 0.0, 0.0));
    }
    vertices->push_back(osg::Vec3(0.0, 0.0, length));
    geom->setVertexArray(vertices);

    osg::Vec4Array* colors = new osg::Vec4Array;
    if (rgbUsed) {
        colors->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0));
        colors->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0));
        colors->push_back(osg::Vec4(0.0, 1.0, 0.0, 1.0));
        colors->push_back(osg::Vec4(0.0, 1.0, 0.0, 1.0));
        colors->push_back(osg::Vec4(0.0, 0.0, 1.0, 1.0));
        colors->push_back(osg::Vec4(0.0, 0.0, 1.0, 1.0));
        geom->setColorArray(colors);
        geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    }
    else {
        colors->push_back(AXIS_COLOR);
        geom->setColorArray(colors);
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    }

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 6));

    osg::StateSet* stateSet = axisGeode->getOrCreateStateSet();
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    return axisGeode.release();
}

osg::Geometry* scene_util::createCircleFloor(float  radius,
                                             int    numSegments,
                                             float  lineWidth,
                                             bool   useStipple)
{
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setName("CircleFloor");

    osg::Vec3Array* vertices = new osg::Vec3Array(numSegments);
    double angle = 2.0 * osg::PI / numSegments;
    for (int i = 0; i < numSegments; ++i) {
        osg::Vec3 pos = osg::Vec3(std::cos(angle * i), std::sin(angle * i), 0.0) * radius;
        (*vertices)[i].set(pos);
    }

    geom->setVertexArray(vertices);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, numSegments));

    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(0.1, 0.1, 0.1, 1.0));
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    osg::StateSet* stateSet = geom->getOrCreateStateSet();
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::LineWidth* width = new osg::LineWidth;
    width->setWidth(lineWidth);
    stateSet->setAttributeAndModes(width, osg::StateAttribute::ON);

    if (useStipple) {
        osg::LineStipple* lineStipple = new osg::LineStipple;
        lineStipple->setFactor(4);
        lineStipple->setPattern(0xaaaa);
        stateSet->setAttributeAndModes(lineStipple, osg::StateAttribute::ON);
    }

    return geom.release();
}

osg::Geometry* scene_util::createStippledLine(const osg::Vec3&  pos0,
                                              const osg::Vec3&  pos1,
                                              const osg::Vec4&  color,
                                              float             width,
                                              GLint             stippleFactor,
                                              GLushort          stipplePattern)
{
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setName("StippledLine");

    osg::Vec3Array* vertices = new osg::Vec3Array;
    vertices->push_back(pos0);
    vertices->push_back(pos1);
    geom->setVertexArray(vertices);

    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(color);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    osg::StateSet* stateSet = geom->getOrCreateStateSet();

    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::Depth* depth = new osg::Depth;
    depth->setFunction(osg::Depth::LESS);
    depth->setRange(0.0, 0.999999999);
    stateSet->setAttributeAndModes(depth, osg::StateAttribute::ON);

    osg::LineWidth* lineWidth = new osg::LineWidth;
    lineWidth->setWidth(width);
    stateSet->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);

    osg::LineStipple* lineStipple = new osg::LineStipple;
    lineStipple->setFactor(stippleFactor);
    lineStipple->setPattern(stipplePattern);
    stateSet->setAttributeAndModes(lineStipple, osg::StateAttribute::ON);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));

    return geom.release();
}

osg::Geometry* scene_util::createArc(const osg::Vec3&   pos0,
                                     const osg::Vec3&   pos1,
                                     int                numSegments,
                                     const osg::Vec4&   color,
                                     float              width,
                                     GLint              stippleFactor,
                                     GLushort           stipplePattern)
{
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setName("ArcLine");

    numSegments = std::max(numSegments, 1);
    int numVerts = numSegments + 1;
    osg::Vec3Array* vertices = new osg::Vec3Array(numVerts);
    for (int i = 0; i < numVerts; ++i) {
        float weight = static_cast<float>(i) / numSegments;
        osg::Vec3 pos = lb::lerp(pos0, pos1, weight);
        float radius = lb::lerp(pos0.length(), pos1.length(), weight);
        pos.normalize();
        pos *= radius;

        (*vertices)[i].set(pos);
    }
    geom->setVertexArray(vertices);

    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(color);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    osg::StateSet* stateSet = geom->getOrCreateStateSet();

    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::LineWidth* lineWidth = new osg::LineWidth;
    lineWidth->setWidth(width);
    stateSet->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);

    osg::LineStipple* lineStipple = new osg::LineStipple;
    lineStipple->setFactor(stippleFactor);
    lineStipple->setPattern(stipplePattern);
    stateSet->setAttributeAndModes(lineStipple, osg::StateAttribute::ON);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, numVerts));

    return geom.release();
}
