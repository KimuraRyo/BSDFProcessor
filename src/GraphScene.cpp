// =================================================================== //
// Copyright (C) 2014-2020 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "GraphScene.h"

#include <osg/ClipPlane>
#include <osg/Depth>
#include <osg/Geometry>
#include <osg/LineStipple>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/PolygonOffset>
#include <osg/PolygonMode>

#include <libbsdf/Brdf/HalfDifferenceCoordinatesBrdf.h>
#include <libbsdf/Brdf/SpecularCoordinatesBrdf.h>
#include <libbsdf/Brdf/SphericalCoordinatesBrdf.h>
#include <libbsdf/Common/SpectrumUtility.h>

#include "SceneUtil.h"
#include "SpecularCenteredCoordinateSystem.h"

const osg::Vec4 RED(1.0, 0.2, 0.2, 1.0);
const osg::Vec4 GREEN(0.2, 1.0, 0.2, 1.0);
const osg::Vec4 BLUE(0.3, 0.3, 1.0, 1.0);
const osg::Vec4 YELLOW(0.8, 0.8, 0.2, 1.0);

constexpr int NUM_ARC_SEGMENTS = 512;

constexpr float RADIUS_FACTOR_1 = 1.0f + 0.005f;
constexpr float RADIUS_FACTOR_2 = 1.0f - 0.005f;

GraphScene::GraphScene() : data_(0),
                           numMultiSamples_(4),
                           oitUsed_(false),
                           numOitPasses_(8),
                           logPlotUsed_(false),
                           baseOfLogarithm_(10.0f),
                           scaleLength1_(0.5f),
                           scaleLength2_(1.0f),
                           scaleInPlaneOfIncidenceUsed_(false),
                           displayMode_(NORMAL_DISPLAY),
                           arcDisplayMode_(ArcDisplayMode::SPHERICAL),
                           arcDisplayUsed_(false),
                           inThetaIndex_(0),
                           inPhiIndex_(0),
                           wavelengthIndex_(0),
                           pickedInDir_(0.0, 0.0, 0.0),
                           pickedOutDir_(0.0, 0.0, 0.0)
{
    const int windowResolution = 512;

    root_ = new osg::Group;
    root_->setName("root_");

    postProcessingChild_ = new osg::Group;
    postProcessingChild_->setName("postProcessingChild_");

    postProcessingGroup_ = createPostProcessing(postProcessingChild_, windowResolution, windowResolution);
    postProcessingGroup_->setName("postProcessingGroup_");

    oitChild_ = new osg::Group;
    oitChild_->setName("oitChild_");

    oitGroup_ = scene_util::createOitGroup(oitChild_, windowResolution, windowResolution, numOitPasses_);
    oitGroup_->setName("oitGroup_");

    scene_ = new osg::Group;
    scene_->setName("scene_");

    bsdfGroup_ = new osg::Group;
    bsdfGroup_->setName("bsdfGroup_");

    brdfMeshGroup_ = new osg::Group;
    attachGraphShader(brdfMeshGroup_);
    brdfMeshGroup_->setName("brdfMeshGroup_");

    brdfPointGroup_ = new osg::Group;
    brdfPointGroup_->setName("brdfPointGroup_");

    brdfTextGroup_ = new osg::Group;
    brdfTextGroup_->setName("brdfTextGroup_");

    specularReflectanceGroup_ = new osg::Group;
    specularReflectanceGroup_->setName("specularReflectanceGroup_");

    accessoryGroup_ = new osg::Group;
    accessoryGroup_->setName("accessoryGroup_");
    accessoryGroup_->setNodeMask(UNDEFINED_MASK);

    axisGroup_ = new osg::Group;
    attachColorShader(axisGroup_);
    axisGroup_->setName("axisGroup_");

    axisTextLabelGroup_ = new osg::Group;
    axisTextLabelGroup_->setName("axisTextLabelGroup_");

    circleGroup_ = new osg::Group;
    attachColorShader(circleGroup_);
    circleGroup_->setName("circleGroup_");

    inOutDirGroup_ = new osg::Group;
    attachColorShader(inOutDirGroup_);
    inOutDirGroup_->setName("inOutDirGroup_");

    scaleInPlaneOfIncidenceGroup_ = new osg::Group;
    attachColorShader(scaleInPlaneOfIncidenceGroup_);
    scaleInPlaneOfIncidenceGroup_->setName("scaleInPlaneOfIncidenceGroup_");

    root_->addChild(postProcessingGroup_);
    postProcessingChild_->addChild(oitGroup_);
    oitChild_->addChild(scene_);

    scene_->addChild(bsdfGroup_);
    bsdfGroup_->addChild(brdfMeshGroup_);
    bsdfGroup_->addChild(brdfPointGroup_);
    bsdfGroup_->addChild(brdfTextGroup_);
    bsdfGroup_->addChild(specularReflectanceGroup_);

    scene_->addChild(accessoryGroup_);
    accessoryGroup_->addChild(axisGroup_);
    accessoryGroup_->addChild(axisTextLabelGroup_);
    accessoryGroup_->addChild(circleGroup_);
    accessoryGroup_->addChild(inOutDirGroup_);
    accessoryGroup_->addChild(scaleInPlaneOfIncidenceGroup_);

    root_->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
}

void GraphScene::updateView(int width, int height)
{
    int numMultiSamples = oitUsed_ ? 0 : numMultiSamples_;
    int numOitPasses = oitUsed_ ? numOitPasses_ : 1;

    // Replace post processing group.
    {
        osg::Group* oldGroup = postProcessingGroup_;
        osg::Group* newGroup = createPostProcessing(postProcessingChild_, width, height, numMultiSamples);
        if (!newGroup) return;

        postProcessingGroup_ = newGroup;

        for (auto parents : oldGroup->getParents()) {
            parents->replaceChild(oldGroup, newGroup);
        }
    }

    // Replace order independent transparency group.
    {
        osg::Group* oldGroup = oitGroup_;
        osg::Group* newGroup = scene_util::createOitGroup(oitChild_, width, height, numOitPasses, numMultiSamples);
        if (!newGroup) return;

        oitGroup_ = newGroup;

        for (auto parents : oldGroup->getParents()) {
            parents->replaceChild(oldGroup, newGroup);
        }
    }
}

void GraphScene::createAxisAndScale()
{
    if (!data_) return;

    // Add axis.
    {
        axisGroup_->removeChildren(0, axisGroup_->getNumChildren());

        osg::Vec3 vec = modifyLineLength(lb::Vec3(1.0, 0.0, 0.0));
        float length = std::max(vec.length(), 2.0f);

        osg::Geode* geode = scene_util::createAxis(length, false);
        axisGroup_->addChild(geode);
    }

    float radius1, radius2;
    if (logPlotUsed_ && isLogPlotAcceptable()) {
        radius1 = scene_util::toLogValue(scaleLength1_, baseOfLogarithm_);
        radius2 = scene_util::toLogValue(scaleLength2_, baseOfLogarithm_);
    }
    else {
        radius1 = scaleLength1_;
        radius2 = scaleLength2_;
    }

    // Add circles.
    {
        circleGroup_->removeChildren(0, circleGroup_->getNumChildren());

        osg::Geode* geode = new osg::Geode;
        circleGroup_->addChild(geode);

        geode->addDrawable(scene_util::createCircleFloor(radius1, NUM_ARC_SEGMENTS, 1.0f, false));
        geode->addDrawable(scene_util::createCircleFloor(radius2, NUM_ARC_SEGMENTS, 1.0f, false));
    }

    axisTextLabelGroup_->removeChildren(0, axisTextLabelGroup_->getNumChildren());

    // Add text labels.
    if (data_->isEmpty()) {
        osg::Geode* geode = new osg::Geode;
        geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        axisTextLabelGroup_->addChild(geode);

        geode->addDrawable(scene_util::createTextLabel("0.5", osg::Vec3(radius1, 0.0, 0.0)));
        geode->addDrawable(scene_util::createTextLabel("0.5", osg::Vec3(-radius1, 0.0, 0.0)));
        geode->addDrawable(scene_util::createTextLabel("0.5", osg::Vec3(0.0, radius1, 0.0)));
        geode->addDrawable(scene_util::createTextLabel("0.5", osg::Vec3(0.0, -radius1, 0.0)));

        geode->addDrawable(scene_util::createTextLabel("1.0", osg::Vec3(radius2, 0.0, 0.0)));
        geode->addDrawable(scene_util::createTextLabel("1.0", osg::Vec3(-radius2, 0.0, 0.0)));
        geode->addDrawable(scene_util::createTextLabel("1.0", osg::Vec3(0.0, radius2, 0.0)));
        geode->addDrawable(scene_util::createTextLabel("1.0", osg::Vec3(0.0, -radius2, 0.0)));
    }
}

void GraphScene::showScaleInPlaneOfIncidence(bool on)
{
    scaleInPlaneOfIncidenceGroup_->removeChildren(0, scaleInPlaneOfIncidenceGroup_->getNumChildren());

    scaleInPlaneOfIncidenceUsed_ = on;

    if (!scaleInPlaneOfIncidenceUsed_ || !data_) return;

    osg::Geode* geode = new osg::Geode;
    scaleInPlaneOfIncidenceGroup_->addChild(geode);

    float inTheta, inPhi;
    lb::SphericalCoordinateSystem::fromXyz(pickedInDir_, &inTheta, &inPhi);

    osg::Vec3 pos0(0.0, 0.0, 1.0);
    if (data_->getBtdf() || data_->getSpecularTransmittances()) {
        pos0 = -pos0;
    }
    osg::Vec3 pos1 = scene_util::toOsg(lb::SphericalCoordinateSystem::toXyz(lb::PI_2_F, inPhi));
    osg::Vec3 pos2 = scene_util::toOsg(lb::SphericalCoordinateSystem::toXyz(lb::PI_2_F, inPhi + lb::PI_F));

    float coeff1, coeff2;
    if (logPlotUsed_ && isLogPlotAcceptable()) {
        coeff1 = scene_util::toLogValue(scaleLength1_, baseOfLogarithm_);
        coeff2 = scene_util::toLogValue(scaleLength2_, baseOfLogarithm_);
    }
    else {
        coeff1 = scaleLength1_;
        coeff2 = scaleLength2_;
    }

    geode->addDrawable(scene_util::createArc(pos0 * coeff1, pos1 * coeff1, NUM_ARC_SEGMENTS, AXIS_COLOR));
    geode->addDrawable(scene_util::createArc(pos0 * coeff1, pos2 * coeff1, NUM_ARC_SEGMENTS, AXIS_COLOR));
    geode->addDrawable(scene_util::createArc(pos0 * coeff2, pos1 * coeff2, NUM_ARC_SEGMENTS, AXIS_COLOR));
    geode->addDrawable(scene_util::createArc(pos0 * coeff2, pos2 * coeff2, NUM_ARC_SEGMENTS, AXIS_COLOR));
}

void GraphScene::updateScaleInPlaneOfIncidence()
{
    showScaleInPlaneOfIncidence(scaleInPlaneOfIncidenceUsed_);
}

void GraphScene::updateGraphGeometry(int inThetaIndex, int inPhiIndex, int wavelengthIndex)
{
    inThetaIndex_ = inThetaIndex;
    inPhiIndex_ = inPhiIndex;
    wavelengthIndex_ = wavelengthIndex;

    if (inThetaIndex != -1 && inPhiIndex != -1) {
        float inTheta = data_->getIncomingPolarAngle(inThetaIndex);
        float inPhi = data_->getIncomingAzimuthalAngle(inPhiIndex);
        pickedInDir_ = lb::SphericalCoordinateSystem::toXyz(inTheta, inPhi);
        
    }
    else if (inThetaIndex != -1) {
        float inTheta, inPhi;
        lb::SphericalCoordinateSystem::fromXyz(pickedInDir_, &inTheta, &inPhi);

        inTheta = data_->getIncomingPolarAngle(inThetaIndex);
        pickedInDir_ = lb::SphericalCoordinateSystem::toXyz(inTheta, inPhi);
    }

    else if (inPhiIndex != -1) {
        float inTheta, inPhi;
        lb::SphericalCoordinateSystem::fromXyz(pickedInDir_, &inTheta, &inPhi);

        inPhi = data_->getIncomingAzimuthalAngle(inPhiIndex);
        pickedInDir_ = lb::SphericalCoordinateSystem::toXyz(inTheta, inPhi);
    }

    clearGraphGeometry();

    if (data_->getBrdf() ||
        data_->getBtdf()) {
        updateBrdfGeometry(inThetaIndex, inPhiIndex, wavelengthIndex);

    }
    else if (data_->getSpecularReflectances() ||
             data_->getSpecularTransmittances()) {
        updateSpecularReflectanceGeometry(inThetaIndex, inPhiIndex, wavelengthIndex);
    }
    else {
        return;
    }
}

void GraphScene::updateGraphGeometry(double inTheta, double inPhi, int wavelengthIndex)
{
    pickedInDir_ = lb::SphericalCoordinateSystem::toXyz(inTheta, inPhi);

    updateGraphGeometry(-1, -1, wavelengthIndex);
}

void GraphScene::updateGraphGeometry()
{
    if (data_->isEmpty()) return;

    GraphScene::DisplayMode dm = getDisplayMode();
    if (dm == GraphScene::SAMPLE_POINTS_DISPLAY ||
        dm == GraphScene::SAMPLE_POINT_LABELS_DISPLAY) {
        updateGraphGeometry(inThetaIndex_, inPhiIndex_, wavelengthIndex_);
    }
    else {
        double inTheta, inPhi;
        lb::SphericalCoordinateSystem::fromXyz(pickedInDir_, &inTheta, &inPhi);
        updateGraphGeometry(inTheta, inPhi, wavelengthIndex_);
    }
}

void setupAngleArcOnGround(osg::Geode*      geode,
                           const osg::Vec3& dir,
                           float            radius,
                           const osg::Vec4& color,
                           GLushort         stipplePattern = 0xcccc)
{
    bool isNormal = (lb::isEqual(dir.x(), 0.0f) &&
                     lb::isEqual(dir.y(), 0.0f));

    // Create an arc.
    {
        osg::Vec3 normal = osg::Vec3(0.0, 0.0, 1.0) * radius;
        osg::Vec3 axis;
        if (isNormal) {
            axis = osg::Vec3(0.0, 1.0, 0.0);
        }
        else {
            axis = normal ^ dir;
            axis.normalize();
        }

        constexpr float lineWidth = 1.0f;
        constexpr GLint stippleFactor = 1;

        osg::Geometry* geom = scene_util::createArc(normal, lb::PI_F, axis, NUM_ARC_SEGMENTS, color,
                                                    lineWidth, stippleFactor, stipplePattern);
        geode->addDrawable(geom);
    }

    // Create a point on the ground.
    {
        osg::Geometry* geom = new osg::Geometry;
        geode->addDrawable(geom);

        osg::Vec3 pos;
        if (isNormal) {
            pos = osg::Vec3(1.0, 0.0, 0.0);
        }
        else {
            pos = osg::Vec3(dir.x(), dir.y(), 0.0);
            pos.normalize();
        }

        pos *= radius;

        osg::Vec3Array* vertices = new osg::Vec3Array;
        vertices->push_back(pos);
        geom->setVertexArray(vertices);
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));

        osg::Vec4Array* colors = new osg::Vec4Array;
        colors->push_back(color);
        geom->setColorArray(colors);
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);

        osg::StateSet* stateSet = geom->getOrCreateStateSet();

        osg::Point* point = new osg::Point(6.0f);
        stateSet->setAttribute(point);
        stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

        osg::Depth* depth = new osg::Depth;
        depth->setFunction(osg::Depth::LESS);
        depth->setRange(0.0, DEPTH_ZFAR_2);
        geom->getOrCreateStateSet()->setAttributeAndModes(depth, osg::StateAttribute::ON);
    }
}

void GraphScene::setupHalfDiffCoordAngles(const lb::Vec3& inDir, const lb::Vec3& outDir, float arcRadius)
{
    bool transparent = (data_->getBtdf() || data_->getSpecularTransmittances());

    lb::Vec3 brdfInDir = inDir;
    if (transparent) {
        brdfInDir.z() = -brdfInDir.z();
    }

    using std::abs;

    lb::Vec3 halfDir;
    if (abs(inDir.z())  < lb::EPSILON_F &&
        abs(outDir.z()) < lb::EPSILON_F) {
        halfDir = transparent ? lb::Vec3(0.0, 0.0, -1.0) : lb::Vec3(0.0, 0.0, 1.0);
    }
    else {
        halfDir = (brdfInDir + outDir) * 0.5;
        halfDir.normalize();
    }
    osg::Vec3 hDir = modifyLineLength(halfDir);

    constexpr float lineWidth = 2.0f;
    constexpr GLint stippleFactor = 1;

    osg::Geode* geode = new osg::Geode;
    inOutDirGroup_->addChild(geode);

    // Create the line of the halfway vector.
    {
        osg::Vec4 color(0.7, 0.1, 1.0, 1.0);
        constexpr GLushort stipplePattern = 0xf8ff;

        osg::Geometry* lineGeom = scene_util::createStippledLine(osg::Vec3(), hDir, color,
                                                                 lineWidth, stippleFactor, stipplePattern);
        geode->addDrawable(lineGeom);

        setupAngleArcOnGround(geode, scene_util::toOsg(halfDir), arcRadius, color);
    }

    if (transparent) {
        osg::Vec4 color(1.0, 0.5, 0.0, 1.0);
        constexpr GLushort stipplePattern = 0xff8f;

        osg::Vec3 iDir = modifyLineLength(brdfInDir);
        osg::Geometry* lineGeom = scene_util::createStippledLine(osg::Vec3(), iDir, color,
                                                                 lineWidth, stippleFactor, stipplePattern);
        geode->addDrawable(lineGeom);
    }

    osg::Vec3 anglePosI(scene_util::toOsg(inDir));
    anglePosI.normalize();
    anglePosI *= arcRadius;
    if (transparent) {
        anglePosI.z() = -anglePosI.z();
    }

    osg::Vec3 anglePosO(scene_util::toOsg(outDir));
    anglePosO.normalize();
    anglePosO *= arcRadius;

    osg::Vec3 anglePosH(hDir);
    anglePosH.normalize();
    anglePosH *= arcRadius;

    osg::Vec3 anglePosN = osg::Vec3(0.0, 0.0, 1.0) * arcRadius;
    if (transparent) {
        anglePosN.z() = -anglePosN.z();
    }

    osg::Geometry* angleGeomHI = scene_util::createArc(anglePosH * RADIUS_FACTOR_2, anglePosI,
                                                       NUM_ARC_SEGMENTS, GREEN, lineWidth);
    geode->addDrawable(angleGeomHI);

    constexpr GLushort stipplePattern = 0xf8f8;
    osg::Geometry* angleGeomHO = scene_util::createArc(anglePosH * RADIUS_FACTOR_2, anglePosO,
                                                       NUM_ARC_SEGMENTS, GREEN, lineWidth, stippleFactor, stipplePattern);
    geode->addDrawable(angleGeomHO);

    osg::Geometry* angleGeomHN = scene_util::createArc(anglePosH * RADIUS_FACTOR_1, anglePosN,
                                                       NUM_ARC_SEGMENTS, RED, lineWidth);
    geode->addDrawable(angleGeomHN);

    double halfTheta, halfPhi, diffTheta, diffPhi;
    data_->getHalfDiffCoordAngles(inDir, outDir, &halfTheta, &halfPhi, &diffTheta, &diffPhi);

    // half azimuthal angle
    const lb::SampleSet* ss = data_->getSampleSet();
    if (ss && !ss->isIsotropic()) {
        osg::Vec3 anglePosX = osg::Vec3(1.0, 0.0, 0.0) * arcRadius;
        const osg::Vec3 axis(0.0, 0.0, 1.0);
        osg::Geometry* angleGeomHalfPhi = scene_util::createArc(anglePosX * RADIUS_FACTOR_1, halfPhi, axis,
                                                                NUM_ARC_SEGMENTS, YELLOW, lineWidth);
        geode->addDrawable(angleGeomHalfPhi);
    }

    // difference azimuthal angle
    if (data_->getBrdf() ||
        data_->getBtdf()) {
        osg::Vec3 axis = transparent ? scene_util::toOsg(halfDir) : scene_util::toOsg(-halfDir);
        osg::Geometry* angleGeomDiffPhi = scene_util::createArc(anglePosI, diffPhi, axis,
                                                                NUM_ARC_SEGMENTS, BLUE, lineWidth);
        geode->addDrawable(angleGeomDiffPhi);
    }
}

void GraphScene::setupSpecularCoordAngles(const lb::Vec3& inDir, const lb::Vec3& outDir, float arcRadius)
{
    bool transparent = (data_->getBtdf() || data_->getSpecularTransmittances());

    double inTheta, inPhi, specTheta, specPhi;
    data_->getSpecularCoordAngles(inDir, outDir, &inTheta, &inPhi, &specTheta, &specPhi);

    lb::Vec3 specDir;
    if (transparent) {
        specDir = -inDir;

        // Offset the specular direction for refraction.
        if (std::shared_ptr<const lb::Btdf> btdf = data_->getBtdf()) {
            lb::Vec3 tempInDir, refractedDir;
            if (auto specBrdf = dynamic_cast<const lb::SpecularCoordinatesBrdf*>(btdf->getBrdf().get())) {
                specBrdf->toXyz(inTheta, inPhi, 0.0f, specPhi, &tempInDir, &refractedDir);
                refractedDir.z() = -refractedDir.z();
                specDir = refractedDir;
            }
        }
    }
    else {
        specDir = lb::reflect(inDir, lb::Vec3(0.0, 0.0, 1.0));
    }

    specDir.normalize();
    osg::Vec3 sDir = modifyLineLength(specDir);

    constexpr float lineWidth = 2.0f;

    osg::Geode* geode = new osg::Geode;
    inOutDirGroup_->addChild(geode);

    // Create the line of the specular direction.
    {
        osg::Vec4 color(0.0, 0.7, 0.8, 1.0);
        constexpr GLint stippleFactor = 1;
        constexpr GLushort stipplePattern = 0xf8ff;

        osg::Geometry* lineGeom = scene_util::createStippledLine(osg::Vec3(), sDir, color,
                                                                 lineWidth, stippleFactor, stipplePattern);
        geode->addDrawable(lineGeom);

        setupAngleArcOnGround(geode, sDir, arcRadius, color);
    }

    osg::Vec3 anglePosI(scene_util::toOsg(inDir));
    anglePosI.normalize();
    anglePosI *= arcRadius;

    osg::Vec3 anglePosO(scene_util::toOsg(outDir));
    anglePosO.normalize();
    anglePosO *= arcRadius;

    osg::Vec3 anglePosS(scene_util::toOsg(specDir));
    anglePosS.normalize();
    anglePosS *= arcRadius;

    osg::Vec3 anglePosN = osg::Vec3(0.0, 0.0, 1.0) * arcRadius;

    // incoming polar angle
    osg::Geometry* angleGeomIN = scene_util::createArc(anglePosI * RADIUS_FACTOR_1, anglePosN,
                                                       NUM_ARC_SEGMENTS, RED, lineWidth);
    geode->addDrawable(angleGeomIN);

    // specular polar angle
    osg::Geometry* angleGeomON = scene_util::createArc(anglePosO * RADIUS_FACTOR_1, anglePosS,
                                                       NUM_ARC_SEGMENTS, GREEN, lineWidth);
    geode->addDrawable(angleGeomON);

    // incoming azimuthal angle
    const lb::SampleSet* ss = data_->getSampleSet();
    if (ss && !ss->isIsotropic()) {
        osg::Vec3 anglePosX = osg::Vec3(1.0, 0.0, 0.0) * arcRadius;
        const osg::Vec3 axis(0.0, 0.0, 1.0);
        osg::Geometry* angleGeomInPhi = scene_util::createArc(anglePosX * RADIUS_FACTOR_1, inPhi, axis,
                                                              NUM_ARC_SEGMENTS, YELLOW, lineWidth);
        geode->addDrawable(angleGeomInPhi);
    }

    // specular azimuthal angle
    {
        osg::Vec3 axis = transparent ? scene_util::toOsg(specDir) : scene_util::toOsg(-specDir);
        osg::Geometry* angleGeomSpecPhi = scene_util::createArc(anglePosO * RADIUS_FACTOR_1, specPhi, axis,
                                                                NUM_ARC_SEGMENTS, BLUE, lineWidth);
        geode->addDrawable(angleGeomSpecPhi);
    }
}

void GraphScene::setupSphericalCoordAngles(const lb::Vec3& inDir, const lb::Vec3& outDir, float arcRadius)
{
    constexpr float lineWidth = 2.0f;

    osg::Vec3 anglePosI(scene_util::toOsg(inDir));
    anglePosI.normalize();
    anglePosI *= arcRadius;

    osg::Vec3 anglePosO(scene_util::toOsg(outDir));
    anglePosO.normalize();
    anglePosO *= arcRadius;

    osg::Vec3 anglePosN = osg::Vec3(0.0, 0.0, 1.0) * arcRadius;

    osg::Geode* geode = new osg::Geode;
    inOutDirGroup_->addChild(geode);

    // incoming polar angle
    osg::Geometry* angleGeomIN = scene_util::createArc(anglePosI * RADIUS_FACTOR_1, anglePosN,
                                                       NUM_ARC_SEGMENTS, RED, lineWidth);
    geode->addDrawable(angleGeomIN);

    // outgoing polar angle
    {
        bool transparent = (data_->getBtdf() || data_->getSpecularTransmittances());

        osg::Vec3 posN = transparent ? osg::Vec3(0.0, 0.0, -1.0) : osg::Vec3(0.0, 0.0, 1.0);
        posN *= arcRadius;

        osg::Geometry* angleGeomON = scene_util::createArc(anglePosO * RADIUS_FACTOR_1, posN,
                                                           NUM_ARC_SEGMENTS, GREEN, lineWidth);
        geode->addDrawable(angleGeomON);
    }

    osg::Vec3 anglePosX = osg::Vec3(1.0, 0.0, 0.0) * arcRadius;

    double inTheta, inPhi, outTheta, outPhi;
    data_->getShericalCoordAngles(inDir, outDir, &inTheta, &inPhi, &outTheta, &outPhi);

    // incoming azimuthal angle
    const lb::SampleSet* ss = data_->getSampleSet();
    if (ss && !ss->isIsotropic()) {
        const osg::Vec3 axis(0.0, 0.0, 1.0);
        osg::Geometry* angleGeomInPhi = scene_util::createArc(anglePosX * RADIUS_FACTOR_2, inPhi, axis,
                                                              NUM_ARC_SEGMENTS, YELLOW, lineWidth);
        geode->addDrawable(angleGeomInPhi);
    }

    // outgoing azimuthal angle
    {
        const osg::Vec3 axis(0.0, 0.0, 1.0);
        osg::Geometry* angleGeomOutPhi = scene_util::createArc(anglePosX * RADIUS_FACTOR_1, outPhi, axis,
                                                               NUM_ARC_SEGMENTS, BLUE, lineWidth);
        geode->addDrawable(angleGeomOutPhi);
    }
}

void GraphScene::updateInOutDirLine(const lb::Vec3& inDir,
                                    const lb::Vec3& outDir)
{
    pickedInDir_ = inDir;

    if (outDir.isZero()) {
        pickedInDir_.setZero();
    }
    else {
        switch (data_->getDataType()) {
            case lb::SPECULAR_REFLECTANCE_DATA:
                pickedOutDir_ = lb::reflect(inDir, lb::Vec3(0.0, 0.0, 1.0));
                break;
            case lb::SPECULAR_TRANSMITTANCE_DATA:
                pickedOutDir_ = -inDir;
                break;
            default:
                pickedOutDir_ = outDir;
                break;
        }
    }

    updateInOutDirLine();
}

void GraphScene::updateInOutDirLine()
{
    if (pickedInDir_.isZero()) return;

    inOutDirGroup_->removeChildren(0, inOutDirGroup_->getNumChildren());

    constexpr float lineWidth = 2.0f;
    constexpr GLint stippleFactor = 1;

    float arcRadius = scaleLength1_;
    if (logPlotUsed_ && isLogPlotAcceptable()) {
        arcRadius = scene_util::toLogValue(arcRadius, baseOfLogarithm_);
    }

    osg::Geode* geode = new osg::Geode;
    inOutDirGroup_->addChild(geode);

    // Update the line of an incoming direction.
    {
        osg::Vec3 dir = modifyLineLength(pickedInDir_);
        osg::Vec4 color(1.0, 0.0, 0.0, 1.0);
        constexpr GLushort stipplePattern = 0x8fff;
        osg::Geometry* geom = scene_util::createStippledLine(osg::Vec3(), dir, color,
                                                             lineWidth, stippleFactor, stipplePattern);
        geode->addDrawable(geom);

        setupAngleArcOnGround(geode, dir, arcRadius, color);
    }

    if (pickedOutDir_.isZero()) return;

    // Update the line of an outgoing direction.
    {
        osg::Vec3 dir = modifyLineLength(pickedOutDir_);
        osg::Vec4 color(0.0, 0.2, 1.0, 1.0);
        constexpr GLushort stipplePattern = 0xff8f;
        osg::Geometry* geom = scene_util::createStippledLine(osg::Vec3(), dir, color,
                                                             lineWidth, stippleFactor, stipplePattern);
        geode->addDrawable(geom);

        setupAngleArcOnGround(geode, dir, arcRadius, color, 0x3333);
    }

    if (arcDisplayUsed_) {
        switch (arcDisplayMode_) {
            case ArcDisplayMode::HALF_DIFF:
                setupHalfDiffCoordAngles(pickedInDir_, pickedOutDir_, arcRadius);
                break;
            case ArcDisplayMode::SPECULAR:
                setupSpecularCoordAngles(pickedInDir_, pickedOutDir_, arcRadius);
                break;
            case ArcDisplayMode::SPHERICAL:
                setupSphericalCoordAngles(pickedInDir_, pickedOutDir_, arcRadius);
                break;
            default:
                break;
        }
    }
}

void GraphScene::updateInDirLine(const lb::Vec3& inDir)
{
    pickedInDir_ = inDir;
    updateInOutDirLine();
}

void GraphScene::clearOutDirLine()
{
    pickedOutDir_.setZero();
    updateInOutDirLine();
}

bool GraphScene::isLogPlotAcceptable()
{
    if (!data_) return true;
    return (data_->getBrdf() || data_->getBtdf() || data_->isEmpty());
}

void GraphScene::setupBrdfGeode()
{
    const lb::SampleSet* ss = data_->getSampleSet();
    if (!ss) return;

    // Set up mesh.
    {
        brdfMeshGroup_->removeChildren(0, brdfMeshGroup_->getNumChildren());

        osg::Geode* geode = new osg::Geode;
        geode->setNodeMask(BRDF_MASK);
        brdfMeshGroup_->addChild(geode);

        osg::ClipPlane* clipPlane = new osg::ClipPlane;
        if (data_->getBrdf()) {
            clipPlane->setClipPlane(0.0, 0.0, 1.0, 0.0);
        }
        else {
            clipPlane->setClipPlane(0.0, 0.0, -1.0, 0.0);
        }
        geode->getOrCreateStateSet()->setAttributeAndModes(clipPlane, osg::StateAttribute::ON);
    }

    // Set up points.
    {
        brdfPointGroup_->removeChildren(0, brdfPointGroup_->getNumChildren());

        osg::Geode* geode = new osg::Geode;
        brdfPointGroup_->addChild(geode);
    }

    // Set up text labels of sample points.
    {
        brdfTextGroup_->removeChildren(0, brdfTextGroup_->getNumChildren());

        osg::Geode* geode = new osg::Geode;
        brdfTextGroup_->addChild(geode);
    }
}

void GraphScene::attachGraphShader(osg::Node* node)
{
    static const char* vertexShaderSource =
    {
        "#version 120\n"
        "\n"
        "varying vec3 position;\n"
        "varying vec3 normal;\n"
        "varying vec4 color;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    vec4 vertex = gl_Vertex;\n"
        "    gl_Position = gl_ModelViewProjectionMatrix * vertex;\n"
        "    gl_ClipVertex = gl_ModelViewMatrix * vertex;\n"
        "    position = vertex.xyz;\n"
        "    normal = gl_Normal;\n"
        "    color = gl_Color;\n"
        "}\n"
    };
    osg::Shader* graphVertexShader = new osg::Shader(osg::Shader::VERTEX, vertexShaderSource);
    graphVertexShader->setName("graphVertexShader");

    static const char* fragmentShaderSource =
    {
        "#version 120\n"
        "\n"
        "varying vec3 position;\n"
        "varying vec3 normal;\n"
        "varying vec4 color;\n"
        "\n"
        "uniform ivec2      viewSize; // Width and height of view\n"
        "uniform bool       oitFirstPass;\n"
        "uniform sampler2D  oitPrevDepthTexture;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    if (!oitFirstPass) {\n"
        "        float prevDepth = texture2D(oitPrevDepthTexture, gl_FragCoord.xy / viewSize).x;\n"
        "        if (gl_FragCoord.z <= prevDepth) {\n"
        "            discard;\n"
        "        }\n"
        "    }\n"
        "\n"
        "    vec4 fragColor = vec4(0.0, 0.0, 0.0, 1.0);\n"
        "    vec3 viewDir = normalize(gl_ModelViewMatrixInverse[3].xyz - position);\n"
        "    vec3 faceNormal = normalize(normal);\n"
        "    fragColor.xyz = abs(vec3(dot(viewDir, faceNormal)));\n"
        "    fragColor *= color;\n"
        "\n"
        "    gl_FragData[0] = fragColor;\n"
        "    gl_FragData[1].x = gl_FragCoord.z;\n"
        "}\n"
    };
    osg::Shader* graphFragmentShader = new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource);
    graphFragmentShader->setName("graphFragmentShader");

    osg::Program* program = new osg::Program;
    program->addShader(graphVertexShader);
    program->addShader(graphFragmentShader);
    node->getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON);
    node->getOrCreateStateSet()->setName("graphShader");
}

void GraphScene::attachColorShader(osg::Node* node)
{
    static const char* vertexShaderSource =
    {
        "#version 120\n"
        "\n"
        "varying vec4 color;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
        "    color = gl_Color;\n"
        "}\n"
    };
    osg::Shader* colorVertexShader = new osg::Shader(osg::Shader::VERTEX, vertexShaderSource);
    colorVertexShader->setName("colorVertexShader");

    static const char* fragmentShaderSource =
    {
        "#version 120\n"
        "\n"
        "varying vec4 color;\n"
        "\n"
        "uniform ivec2      viewSize; // Width and height of view\n"
        "uniform bool       oitFirstPass;\n"
        "uniform sampler2D  oitPrevDepthTexture;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    if (!oitFirstPass) {\n"
        "        float prevDepth = texture2D(oitPrevDepthTexture, gl_FragCoord.xy / viewSize).x;\n"
        "        if (gl_FragCoord.z <= prevDepth) {\n"
        "            discard;\n"
        "        }\n"
        "    }\n"
        "\n"
        "    vec4 fragColor = color;\n"
        "\n"
        "    gl_FragData[0] = fragColor;\n"
        "    gl_FragData[1].x = gl_FragCoord.z;\n"
        "}\n"
    };
    osg::Shader* colorFragmentShader = new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource);
    colorFragmentShader->setName("colorFragmentShader");

    osg::Program* program = new osg::Program;
    program->addShader(colorVertexShader);
    program->addShader(colorFragmentShader);
    node->getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON);
    node->getOrCreateStateSet()->setName("oitColorDepthShader");
}

osg::Group* GraphScene::createPostProcessing(osg::Node* subgraph, int width, int height, int numFboSamples)
{
    static const char* shaderSource =
    {
        "#version 120\n"
        "\n"
        "uniform sampler2D renderedTexture;\n"
        "uniform float gamma;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    vec2 uv0 = gl_TexCoord[0].xy;\n"
        "    vec4 fragColor = texture2D(renderedTexture, uv0);\n"
        "    fragColor.xyz = pow(fragColor.xyz, vec3(1.0 / gamma));\n"
        "    gl_FragColor = vec4(fragColor.xyz, 1.0);\n"
        "}\n"
    };

    osg::ref_ptr<osg::Shader> postProcessingShader = new osg::Shader(osg::Shader::FRAGMENT, shaderSource);
    postProcessingShader->setName("postProcessingShader");

    osg::Group* postProcessingGroup = scene_util::createPostProcessingGroup(subgraph,
                                                                            width, height,
                                                                            false, true,
                                                                            postProcessingShader.get(),
                                                                            0, 0,
                                                                            numFboSamples, 0);

    if (!postProcessingGroup) {
        lbError << "[GraphScene::createPostProcessing] Failed to create a post-processing group.";
        return 0;
    }

    osg::Uniform* renderedTexUniform = new osg::Uniform("renderedTexture", 0);
    postProcessingGroup->getOrCreateStateSet()->addUniform(renderedTexUniform);

    const float gamma = 2.2f;
    osg::Uniform* gammaUniform = new osg::Uniform("gamma", gamma);
    postProcessingGroup->getOrCreateStateSet()->addUniform(gammaUniform);

    return postProcessingGroup;
}

osg::Vec3 GraphScene::modifyLineLength(const lb::Vec3& pos)
{
    osg::Vec3 newPos(pos[0], pos[1], pos[2]);

    if (data_->getBrdf() || data_->getBtdf()) {
        float val = data_->getMaxValuesPerWavelength().maxCoeff();
        if (logPlotUsed_) {
            val = scene_util::toLogValue(val, baseOfLogarithm_);
        }

        const float coeff = 1.2f;
        if (val * coeff > 1.0f) {
            newPos *= std::min(val * coeff, 100.0f);
        }
    }

    return newPos;
}

void GraphScene::updateBrdfGeometry(int inThetaIndex, int inPhiIndex, int wavelengthIndex)
{
    const lb::SampleSet* ss = data_->getSampleSet();
    if (!ss) return;

    setupBrdfGeode();

    bool paramValid = (inThetaIndex < data_->getNumInTheta() &&
                       inPhiIndex   < data_->getNumInPhi() &&
                       wavelengthIndex < ss->getNumWavelengths());
    if (!paramValid) return;

    double inTheta, inPhi;
    lb::SphericalCoordinateSystem::fromXyz(pickedInDir_, &inTheta, &inPhi);

    const lb::Brdf* brdf = data_->getBrdfData();
    if (!brdf) return;

    lb::DataType dataType = data_->getDataType();

    osg::Geode* meshGeode = brdfMeshGroup_->getChild(0)->asGeode();
    meshGeode->getOrCreateStateSet()->removeAttribute(osg::StateAttribute::POLYGONOFFSET);

    switch (displayMode_) {
        case PHOTOMETRY_DISPLAY: {
            setupBrdfMeshGeometry(*brdf, inTheta, inPhi, wavelengthIndex, dataType, true);
            updateInOutDirLine();

            oitUsed_ = false;
            break;
        }
        case NORMAL_DISPLAY: {
            setupBrdfMeshGeometry(*brdf, inTheta, inPhi, wavelengthIndex, dataType);
            updateInOutDirLine();

            oitUsed_ = false;
            break;
        }
        case ALL_INCOMING_POLAR_ANGLES_DISPLAY: {
            inOutDirGroup_->removeChildren(0, inOutDirGroup_->getNumChildren());

            double curInTheta;
            #pragma omp parallel for private(curInTheta)
            for (int i = 0; i < data_->getNumInTheta(); ++i) {
                curInTheta = data_->getIncomingPolarAngle(i);
                setupBrdfMeshGeometry(*brdf, curInTheta, inPhi, wavelengthIndex, dataType);
            }

            for (int i = 0; i < data_->getNumInTheta(); ++i) {
                curInTheta = data_->getIncomingPolarAngle(i);
                double    inThetaRatio = curInTheta / lb::PI_2_D;
                osg::Vec4 color(scene_util::hueToRgb(inThetaRatio), 1.0);

                lb::Vec3 curInDir = lb::SphericalCoordinateSystem::toXyz(curInTheta, inPhi);
                osg::Vec3 dir = modifyLineLength(curInDir);
                osg::Geometry* geom = scene_util::createStippledLine(osg::Vec3(), dir, color, 2.0f);

                osg::Geode* geode = new osg::Geode;
                geode->addDrawable(geom);
                inOutDirGroup_->addChild(geode);
            }

            oitUsed_ = true;
            break;
        }
        case ALL_INCOMING_AZIMUTHAL_ANGLES_DISPLAY: {
            inOutDirGroup_->removeChildren(0, inOutDirGroup_->getNumChildren());

            double endInPhi = data_->getIncomingAzimuthalAngle(data_->getNumInPhi() - 1) -
                              lb::SphericalCoordinateSystem::MAX_ANGLE1;
            bool duplicate = lb::isEqual(data_->getIncomingAzimuthalAngle(0), endInPhi);
            int numInPhi = duplicate ? data_->getNumInPhi() - 1 : data_->getNumInPhi();
            double curInPhi;
            #pragma omp parallel for private(curInPhi)
            for (int i = 0; i < numInPhi; ++i) {
                curInPhi = data_->getIncomingAzimuthalAngle(i);
                setupBrdfMeshGeometry(*brdf, inTheta, curInPhi, wavelengthIndex, dataType);
            }

            for (int i = 0; i < numInPhi; ++i) {
                curInPhi = data_->getIncomingAzimuthalAngle(i);
                double    inPhiRatio = curInPhi / lb::TAU_D;
                osg::Vec4 color(scene_util::hueToRgb(inPhiRatio), 1.0);

                lb::Vec3 curInDir = lb::SphericalCoordinateSystem::toXyz(inTheta, curInPhi);
                osg::Vec3 dir = modifyLineLength(curInDir);
                osg::Geometry* geom = scene_util::createStippledLine(osg::Vec3(), dir, color, 2.0f);

                osg::Geode* geode = new osg::Geode;
                geode->addDrawable(geom);
                inOutDirGroup_->addChild(geode);
            }

            oitUsed_ = true;
            break;
        }
        case ALL_WAVELENGTHS_DISPLAY: {
            #pragma omp parallel for
            for (int i = 0; i < data_->getNumWavelengths(); ++i) {
                setupBrdfMeshGeometry(*brdf, inTheta, inPhi, i, dataType);
            }
            updateInOutDirLine();
            oitUsed_ = true;
            break;
        }
        case SAMPLE_POINTS_DISPLAY: {
            setupBrdfMeshGeometry(*brdf, inTheta, inPhi, wavelengthIndex, dataType);
            meshGeode->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonOffset(1.0f, 1.0f),
                                                                        osg::StateAttribute::ON);
            if (!data_->isInDirDependentCoordinateSystem()) break;

            // Update point geometry.
            osg::Geometry* geom = scene_util::createBrdfPointGeometry(*brdf,
                                                                      inThetaIndex, inPhiIndex, wavelengthIndex,
                                                                      logPlotUsed_, baseOfLogarithm_, dataType);
            osg::Geode* geode = brdfPointGroup_->getChild(0)->asGeode();
            geode->addDrawable(geom);

            updateInOutDirLine();

            oitUsed_ = false;
            break;
        }
        case SAMPLE_POINT_LABELS_DISPLAY: {
            if (!data_->isInDirDependentCoordinateSystem()) break;

            osg::Geode* geode = brdfTextGroup_->getChild(0)->asGeode();
            scene_util::attachBrdfTextLabels(geode,
                                             *brdf,
                                             inThetaIndex, inPhiIndex, wavelengthIndex,
                                             logPlotUsed_, baseOfLogarithm_, dataType);

            updateInOutDirLine();

            oitUsed_ = false;
            break;
        }
        default: {
            assert(false);
        }
    }
}

void GraphScene::setupBrdfMeshGeometry(const lb::Brdf& brdf, float inTheta, float inPhi, int wavelengthIndex,
                                       lb::DataType dataType, bool photometric)
{
    bool many = ((displayMode_ == ALL_INCOMING_POLAR_ANGLES_DISPLAY && data_->getNumInTheta() > 10) ||
                 (displayMode_ == ALL_INCOMING_AZIMUTHAL_ANGLES_DISPLAY && data_->getNumInPhi() > 10) ||
                 (displayMode_ == ALL_WAVELENGTHS_DISPLAY && data_->getNumWavelengths() > 10));
    int numTheta, numPhi;
    if (many) {
        numTheta = 181;
        numPhi = 91;
    }
    else {
        numTheta = 361;
        numPhi = 361;
    }

    osg::Geometry* meshGeom;
    if (dynamic_cast<const lb::SphericalCoordinatesBrdf*>(&brdf)) {
        meshGeom = scene_util::createBrdfMeshGeometry<lb::SphericalCoordinateSystem>(
            brdf, inTheta, inPhi, wavelengthIndex,
            logPlotUsed_, baseOfLogarithm_, dataType, photometric,
            numTheta, numPhi);
    }
    else if (dynamic_cast<const lb::SpecularCoordinatesBrdf*>(&brdf)) {
        meshGeom = scene_util::createBrdfMeshGeometry<lb::SpecularCoordinateSystem>(
            brdf, inTheta, inPhi, wavelengthIndex,
            logPlotUsed_, baseOfLogarithm_, dataType, photometric,
            numTheta, numPhi);
    }
    else {
        meshGeom = scene_util::createBrdfMeshGeometry<SpecularCenteredCoordinateSystem>(
            brdf, inTheta, inPhi, wavelengthIndex,
            logPlotUsed_, baseOfLogarithm_, dataType, photometric,
            numTheta, numPhi);
    }

    // Set the color of graph.
    const float alpha = 0.3f;
    osg::Vec4 color;
    bool colorUpdate = false;
    if (displayMode_ == ALL_INCOMING_POLAR_ANGLES_DISPLAY && data_->getNumInTheta() > 1) {
        float inThetaRatio = inTheta / lb::PI_2_F;
        color = osg::Vec4(scene_util::hueToRgb(inThetaRatio), alpha);
        colorUpdate = true;
    }
    else if (displayMode_ == ALL_INCOMING_AZIMUTHAL_ANGLES_DISPLAY && data_->getNumInPhi() > 1) {
        float inPhiRatio = inPhi / lb::TAU_F;
        color = osg::Vec4(scene_util::hueToRgb(inPhiRatio), alpha);
        colorUpdate = true;
    }
    else if (displayMode_ == ALL_WAVELENGTHS_DISPLAY &&
             data_->getNumWavelengths() > 1) {
        const lb::SampleSet* ss = brdf.getSampleSet();

        if (ss->getColorModel() == lb::RGB_MODEL ||
            ss->getColorModel() == lb::XYZ_MODEL) {
            if (wavelengthIndex == 0) {
                color = osg::Vec4(1.0, 0.0, 0.0, alpha);
            }
            else if (wavelengthIndex == 1) {
                color = osg::Vec4(0.0, 1.0, 0.0, alpha);
            }
            else if (wavelengthIndex == 2) {
                color = osg::Vec4(0.0, 0.0, 1.0, alpha);
            }
        }
        else {
            const lb::Arrayf& wls = ss->getWavelengths();
            lb::Vec3 rgb = lb::SpectrumUtility::wavelengthToSrgb(wls[wavelengthIndex]);
            color = osg::Vec4(scene_util::toOsg(rgb), alpha);
        }
        colorUpdate = true;
    }

    if (colorUpdate) {
        osg::Vec4Array* colors = new osg::Vec4Array;
        colors->push_back(color);
        meshGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
    }

    if (osg::Geode* geode = brdfMeshGroup_->getChild(0)->asGeode()) {
        #pragma omp critical
        geode->addDrawable(meshGeom);
    }
}

void GraphScene::updateSpecularReflectanceGeometry(int inThetaIndex, int inPhiIndex, int wavelengthIndex)
{
    const lb::SampleSet2D* ss2;
    if (data_->getSpecularReflectances()) {
        ss2 = data_->getSpecularReflectances().get();
    }
    else if (data_->getSpecularTransmittances()) {
        ss2 = data_->getSpecularTransmittances().get();
    }
    else {
        return;
    }

    bool paramValid = (inThetaIndex < ss2->getNumTheta() &&
                       inPhiIndex   < ss2->getNumPhi() &&
                       wavelengthIndex < ss2->getNumWavelengths());
    if (!paramValid) return;

    specularReflectanceGroup_->removeChildren(0, specularReflectanceGroup_->getNumChildren());

    osg::Geode* geode = new osg::Geode;
    geode->setNodeMask(SPECULAR_REFLECTANCE_MASK);
    specularReflectanceGroup_->addChild(geode);

    const lb::Spectrum& sp = ss2->getSpectrum(pickedInDir_);
    float value;
    if (displayMode_ == PHOTOMETRY_DISPLAY) {
        value = lb::SpectrumUtility::spectrumToY(sp, ss2->getColorModel(), ss2->getWavelengths());
    }
    else {
        value = sp[wavelengthIndex];
    }

    // Update specular reflectance.
    lb::Vec3 outDir;
    if (data_->getSpecularReflectances()) {
        outDir = lb::reflect(pickedInDir_, lb::Vec3(0.0, 0.0, 1.0));
    }
    else {
        outDir = -pickedInDir_;
    }
    lb::Vec3 outPos = outDir * value;
    osg::Vec3Array* vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3());
    vertices->push_back(osg::Vec3(outPos[0], outPos[1], outPos[2]));
    osg::Geometry* spRefGeometry = new osg::Geometry;
    spRefGeometry->setVertexArray(vertices);
    geode->addDrawable(spRefGeometry);

    osg::DrawElementsUByte* spRefLine = new osg::DrawElementsUByte(osg::PrimitiveSet::LINES, 0);
    spRefLine->push_back(0);
    spRefLine->push_back(1);
    spRefGeometry->addPrimitiveSet(spRefLine);

    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    spRefGeometry->setColorArray(colors);
    spRefGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);

    osg::StateSet* stateSet = spRefGeometry->getOrCreateStateSet();
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::Depth* depth = new osg::Depth;
    depth->setFunction(osg::Depth::LESS);
    depth->setRange(0.0, DEPTH_ZFAR_2);
    stateSet->setAttributeAndModes(depth, osg::StateAttribute::ON);

    osg::LineWidth* lineWidth = new osg::LineWidth;
    lineWidth->setWidth(4.0f);
    stateSet->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);
}

void GraphScene::clearGraphGeometry()
{
    brdfMeshGroup_->removeChildren(0, brdfMeshGroup_->getNumChildren());
    brdfPointGroup_->removeChildren(0, brdfPointGroup_->getNumChildren());
    brdfTextGroup_->removeChildren(0, brdfTextGroup_->getNumChildren());
    specularReflectanceGroup_->removeChildren(0, specularReflectanceGroup_->getNumChildren());
}
