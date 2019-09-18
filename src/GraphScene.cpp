// =================================================================== //
// Copyright (C) 2014-2019 Kimura Ryo                                  //
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
#include <libbsdf/Common/Log.h>
#include <libbsdf/Common/SpectrumUtility.h>
#include <libbsdf/Common/SpecularCoordinateSystem.h>
#include <libbsdf/Common/SphericalCoordinateSystem.h>

#include "SpecularCenteredCoordinateSystem.h"

GraphScene::GraphScene() : data_(0),
                           numMultiSamples_(4),
                           oitUsed_(false),
                           numOitPasses_(8),
                           logPlotUsed_(false),
                           baseOfLogarithm_(10.0f),
                           scaleLength1_(0.5f),
                           scaleLength2_(1.0f),
                           displayMode_(NORMAL_DISPLAY),
                           camera_(0),
                           cameraManipulator_(0),
                           inThetaIndex_(0),
                           inPhiIndex_(0),
                           wavelengthIndex_(0),
                           inTheta_(0.0f),
                           inPhi_(0.0f),
                           pickedInDir_(0.0, 0.0, 0.0),
                           pickedOutDir_(0.0, 0.0, 0.0)
{
    const int windowResolution = 512;

    root_ = new osg::Group;
    root_->setName("root_");

    postProcessingChild_ = new osg::Group;
    postProcessingChild_->setName("postProcessingChild_");

    postProcessingGroup_ = createPostProcessing(postProcessingChild_.get(), windowResolution, windowResolution);
    postProcessingGroup_->setName("postProcessingGroup_");

    oitChild_ = new osg::Group;
    oitChild_->setName("oitChild_");

    oitGroup_ = scene_util::createOitGroup(oitChild_.get(), windowResolution, windowResolution, numOitPasses_);
    oitGroup_->setName("oitGroup_");

    scene_ = new osg::Group;
    scene_->setName("scene_");

    bsdfGroup_ = new osg::Group;
    bsdfGroup_->setName("bsdfGroup_");

    accessoryGroup_ = new osg::Group;
    accessoryGroup_->setName("accessoryGroup_");
    accessoryGroup_->setNodeMask(UNDEFINED_MASK);

    root_->addChild(postProcessingGroup_.get());
    postProcessingChild_->addChild(oitGroup_.get());
    oitChild_->addChild(scene_.get());
    scene_->addChild(bsdfGroup_.get());
    scene_->addChild(accessoryGroup_.get());

    initializeInDirLine();
    initializeInOutDirLine();
}

void GraphScene::updateView(int width, int height)
{
    assert(postProcessingGroup_.valid() && postProcessingChild_.valid());
    assert(oitGroup_.valid() && oitChild_.valid());

    int numMultiSamples = oitUsed_ ? 0 : numMultiSamples_;
    int numOitPasses = oitUsed_ ? numOitPasses_ : 1;

    // Replace post processing group.
    {
        osg::Group* oldGroup = postProcessingGroup_.get();
        osg::Group* newGroup = createPostProcessing(postProcessingChild_.get(), width, height, numMultiSamples);
        if (!newGroup) return;

        postProcessingGroup_ = newGroup;

        osg::Node::ParentList parents = oldGroup->getParents();
        for (auto it = parents.begin(); it != parents.end(); ++it) {
            (*it)->replaceChild(oldGroup, postProcessingGroup_.get());
        }
    }

    // Replace order independent transparency group.
    {
        osg::Group* oldGroup = oitGroup_.get();
        osg::Group* newGroup = scene_util::createOitGroup(oitChild_.get(), width, height, numOitPasses, numMultiSamples);
        if (!newGroup) return;

        oitGroup_ = newGroup;

        osg::Node::ParentList parents = oldGroup->getParents();
        for (auto it = parents.begin(); it != parents.end(); ++it) {
            (*it)->replaceChild(oldGroup, oitGroup_.get());
        }
    }
}

void GraphScene::createAxisAndScale()
{
    if (!data_) return;

    // Add axis.
    {
        if (axisGeode_.valid()) {
            accessoryGroup_->removeChild(axisGeode_.get());
        }

        bool backSideShown;
        if (data_->getBtdf() || data_->getSpecularTransmittances()) {
            backSideShown = true;
        }
        else {
            backSideShown = false;
        }

        osg::Vec3 tempAxis = modifyLineLength(lb::Vec3(1.0, 0.0, 0.0));
        float axisSize = std::max(tempAxis.length(), 2.0f);

        axisGeode_ = scene_util::createAxis(axisSize, backSideShown, false);
        axisGeode_->setName("axisGeode_");
        attachColorShader(axisGeode_);
        accessoryGroup_->addChild(axisGeode_);
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
        if (circleGeode_.valid()) {
            accessoryGroup_->removeChild(circleGeode_.get());
        }

        circleGeode_ = new osg::Geode;
        circleGeode_->setName("circleGeode_");
        attachColorShader(circleGeode_);
        accessoryGroup_->addChild(circleGeode_);

        circleGeode_->addDrawable(scene_util::createCircleFloor(radius1, 256, 1.0f, false));
        circleGeode_->addDrawable(scene_util::createCircleFloor(radius2, 256, 1.0f, false));
    }

    if (axisTextLabelGeode_.valid()) {
        accessoryGroup_->removeChild(axisTextLabelGeode_.get());
    }

    // Add text labels.
    if (data_->isEmpty()) {
        axisTextLabelGeode_ = new osg::Geode;
        axisTextLabelGeode_->setName("axisTextLabelGeode_");
        axisTextLabelGeode_->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        accessoryGroup_->addChild(axisTextLabelGeode_);

        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("0.5", osg::Vec3(radius1, 0.0, 0.0)));
        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("0.5", osg::Vec3(-radius1, 0.0, 0.0)));
        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("0.5", osg::Vec3(0.0, radius1, 0.0)));
        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("0.5", osg::Vec3(0.0, -radius1, 0.0)));

        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("1.0", osg::Vec3(radius2, 0.0, 0.0)));
        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("1.0", osg::Vec3(-radius2, 0.0, 0.0)));
        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("1.0", osg::Vec3(0.0, radius2, 0.0)));
        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("1.0", osg::Vec3(0.0, -radius2, 0.0)));
    }
}

void GraphScene::showScaleInPlaneOfIncidence(bool on)
{
    if (!data_) return;

    if (scaleInPlaneOfIncidenceGeode_.valid()) {
        accessoryGroup_->removeChild(scaleInPlaneOfIncidenceGeode_.get());
    }

    if (!on) return;

    scaleInPlaneOfIncidenceGeode_ = new osg::Geode;
    scaleInPlaneOfIncidenceGeode_->setName("scaleInPlaneOfIncidenceGeode_");
    attachColorShader(scaleInPlaneOfIncidenceGeode_);
    accessoryGroup_->addChild(scaleInPlaneOfIncidenceGeode_);

    osg::Vec3 pos0(0.0, 0.0, 1.0);
    if (data_->getBtdf() || data_->getSpecularTransmittances()) {
        pos0 = -pos0;
    }
    osg::Vec3 pos1 = scene_util::toOsg(lb::SphericalCoordinateSystem::toXyz(lb::PI_2_F, inPhi_));
    osg::Vec3 pos2 = scene_util::toOsg(lb::SphericalCoordinateSystem::toXyz(lb::PI_2_F, inPhi_ + lb::PI_F));

    float coeff1, coeff2;
    if (logPlotUsed_ && isLogPlotAcceptable()) {
        coeff1 = scene_util::toLogValue(scaleLength1_, baseOfLogarithm_);
        coeff2 = scene_util::toLogValue(scaleLength2_, baseOfLogarithm_);
    }
    else {
        coeff1 = scaleLength1_;
        coeff2 = scaleLength2_;
    }

    scaleInPlaneOfIncidenceGeode_->addDrawable(scene_util::createArc(pos0 * coeff1, pos1 * coeff1, 64, AXIS_COLOR));
    scaleInPlaneOfIncidenceGeode_->addDrawable(scene_util::createArc(pos0 * coeff1, pos2 * coeff1, 64, AXIS_COLOR));
    scaleInPlaneOfIncidenceGeode_->addDrawable(scene_util::createArc(pos0 * coeff2, pos1 * coeff2, 64, AXIS_COLOR));
    scaleInPlaneOfIncidenceGeode_->addDrawable(scene_util::createArc(pos0 * coeff2, pos2 * coeff2, 64, AXIS_COLOR));
}

void GraphScene::updateScaleInPlaneOfIncidence()
{
    if (scaleInPlaneOfIncidenceGeode_.valid()) {
        showScaleInPlaneOfIncidence(true);
    }
}

void GraphScene::createBrdfGeode()
{
    const lb::SampleSet* ss = data_->getSampleSet();
    if (!ss) return;

    if (bxdfMeshGeode_.valid()) {
        bsdfGroup_->removeChild(bxdfMeshGeode_.get());
    }

    // Set up mesh.
    {
        bxdfMeshGeode_ = new osg::Geode;
        bxdfMeshGeode_->setName("bxdfMeshGeode_");
        bxdfMeshGeode_->setNodeMask(BRDF_MASK);
        bsdfGroup_->addChild(bxdfMeshGeode_.get());

        osg::ClipPlane* clipPlane = new osg::ClipPlane;
        if (data_->getBrdf()) {
            clipPlane->setClipPlane(0.0, 0.0, 1.0, -0.000001);
        }
        else {
            clipPlane->setClipPlane(0.0, 0.0, -1.0, -0.000001);
        }
        bxdfMeshGeode_->getOrCreateStateSet()->setAttributeAndModes(clipPlane, osg::StateAttribute::ON);

        attachGraphShader(bxdfMeshGeode_.get());
    }

    // Set up points.
    {
        if (bxdfPointGeode_.valid()) {
            bsdfGroup_->removeChild(bxdfPointGeode_.get());
        }

        bxdfPointGeode_ = new osg::Geode;
        bxdfPointGeode_->setName("bxdfPointGeode_");
        bsdfGroup_->addChild(bxdfPointGeode_.get());
    }

    // Set up text labels of sample points.
    {
        if (bxdfTextGeode_.valid()) {
            bsdfGroup_->removeChild(bxdfTextGeode_.get());
        }

        bxdfTextGeode_ = new osg::Geode;
        bxdfTextGeode_->setName("bxdfTextGeode_");
        bsdfGroup_->addChild(bxdfTextGeode_.get());
    }
}

void GraphScene::updateGraphGeometry(int inThetaIndex, int inPhiIndex, int wavelengthIndex)
{
    inThetaIndex_ = inThetaIndex;
    inPhiIndex_ = inPhiIndex;
    wavelengthIndex_ = wavelengthIndex;

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

void GraphScene::updateGraphGeometry(float inTheta, float inPhi, int wavelengthIndex)
{
    inTheta_ = inTheta;
    inPhi_ = inPhi;

    updateGraphGeometry(-1, -1, wavelengthIndex);
}

void GraphScene::updateGraphGeometry()
{
    GraphScene::DisplayMode dm = getDisplayMode();
    if (dm == GraphScene::SAMPLE_POINTS_DISPLAY ||
        dm == GraphScene::SAMPLE_POINT_LABELS_DISPLAY) {
        updateGraphGeometry(inThetaIndex_, inPhiIndex_, wavelengthIndex_);
    }
    else {
        updateGraphGeometry(inTheta_, inPhi_, wavelengthIndex_);
    }
}

void GraphScene::updateInOutDirLine(const lb::Vec3& inDir,
                                    const lb::Vec3& outDir,
                                    int             wavelengthIndex)
{
    pickedInDir_ = inDir;
    pickedOutDir_ = outDir;
    wavelengthIndex_ = wavelengthIndex;

    inOutDirGeode_->removeDrawables(0, inOutDirGeode_->getNumDrawables());

    if (inDir.isZero() && outDir.isZero()) {
        return;
    }

    const float lineWidth = 2.0f;
    const GLint stippleFactor = 1;

    float arcRadius = std::min(scaleLength1_, scaleLength2_);
    if (logPlotUsed_){
        arcRadius = scene_util::toLogValue(arcRadius, baseOfLogarithm_);
    }

    osg::Depth* depth = new osg::Depth;
    depth->setFunction(osg::Depth::LESS);
    depth->setRange(0.0, 0.9999999);

    // Update the line of an incoming direction.
    {
        osg::Vec4 color(1.0, 0.2, 0.0, 1.0);
        GLushort stipplePattern = 0x8fff;

        osg::Vec3 dir = modifyLineLength(inDir);
        osg::Geometry* geom = scene_util::createStippledLine(osg::Vec3(), dir, color,
                                                             lineWidth, stippleFactor, stipplePattern);
        inOutDirGeode_->addDrawable(geom);

        osg::Vec3 anglePos0(dir);
        anglePos0.normalize();
        anglePos0 *= arcRadius;

        osg::Vec3 anglePos1(dir.x(), dir.y(), 0.0);
        anglePos1.normalize();
        anglePos1 *= arcRadius;

        osg::Geometry* angleGeom = scene_util::createArc(anglePos0, anglePos1, 64, color);
        angleGeom->getOrCreateStateSet()->setAttributeAndModes(depth, osg::StateAttribute::ON);
        inOutDirGeode_->addDrawable(angleGeom);
    }

    // Update the line of an outgoing direction.
    {
        osg::Vec3 dir = modifyLineLength(outDir);
        if (data_->getBtdf() ||
            data_->getSpecularTransmittances()) {
            dir.z() = -dir.z();
        }

        osg::Vec4 color(0.0, 0.2, 1.0, 1.0);
        GLushort stipplePattern = 0xff8f;

        osg::Geometry* geom = scene_util::createStippledLine(osg::Vec3(), dir, color,
                                                             lineWidth, stippleFactor, stipplePattern);
        inOutDirGeode_->addDrawable(geom);

        osg::Vec3 anglePos0(dir);
        anglePos0.normalize();
        anglePos0 *= arcRadius;

        osg::Vec3 anglePos1(dir.x(), dir.y(), 0.0);
        anglePos1.normalize();
        anglePos1 *= arcRadius;

        osg::Geometry* angleGeom = scene_util::createArc(anglePos0, anglePos1, 64, color);
        angleGeom->getOrCreateStateSet()->setAttributeAndModes(depth, osg::StateAttribute::ON);
        inOutDirGeode_->addDrawable(angleGeom);
    }
}

void GraphScene::updateInOutDirLine()
{
    updateInOutDirLine(pickedInDir_, pickedOutDir_, wavelengthIndex_);
}

void GraphScene::setCamera(osg::Camera* camera)
{
    camera_ = camera;
    camera_->setNearFarRatio(0.00005);
}

bool GraphScene::isLogPlotAcceptable()
{
    if (!data_) return true;
    return (data_->getBrdf() || data_->getBtdf() || data_->isEmpty());
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

void GraphScene::initializeInDirLine()
{
    if (inDirGeode_.valid()) {
        accessoryGroup_->removeChild(inDirGeode_.get());
    }

    inDirGeode_ = new osg::Geode;
    inDirGeode_->setName("inDirGeode_");
    attachColorShader(inDirGeode_.get());
    accessoryGroup_->addChild(inDirGeode_.get());
}

void GraphScene::updateInDirLine(const lb::Vec3& inDir)
{
    osg::Vec3 dir;
    if (inDir[2] >= 0.0) {
        dir = modifyLineLength(inDir);
    }
    else {
        dir = osg::Vec3(0.0, 0.0, 1.0);
        lbError << "[GraphScene::updateInDirLine] Negative Z-component: " << inDir;
    }

    osg::Geometry* geom = scene_util::createStippledLine(osg::Vec3(), dir,
                                                         osg::Vec4(1.0, 0.0, 0.0, 1.0));
    inDirGeode_->removeDrawables(0, inDirGeode_->getNumDrawables());
    inDirGeode_->addDrawable(geom);
}

void GraphScene::initializeInOutDirLine()
{
    if (inOutDirGeode_.valid()) {
        accessoryGroup_->removeChild(inOutDirGeode_.get());
    }

    inOutDirGeode_ = new osg::Geode;
    inOutDirGeode_->setName("inOutDirGeode_");
    attachColorShader(inOutDirGeode_.get());
    accessoryGroup_->addChild(inOutDirGeode_.get());
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

    bool nodeValid = (bxdfMeshGeode_.valid() &&
                      bxdfPointGeode_.valid() &&
                      bxdfTextGeode_.valid());
    bool paramValid = (inThetaIndex < data_->getNumInTheta() &&
                       inPhiIndex   < data_->getNumInPhi() &&
                       wavelengthIndex < ss->getNumWavelengths());
    if (!nodeValid || !paramValid) return;

    if (inThetaIndex != -1) {
        inTheta_ = data_->getIncomingPolarAngle(inThetaIndex);
    }

    if (inPhiIndex != -1) {
        inPhi_ = data_->getIncomingAzimuthalAngle(inPhiIndex);
    }

    // Update the line of incoming direction.
    lb::Vec3 inDir = lb::SphericalCoordinateSystem::toXyz(inTheta_, inPhi_);
    updateInDirLine(inDir);

    lb::Brdf* brdf;
    lb::DataType dataType;
    if (data_->getBrdf()) {
        brdf = data_->getBrdf();
        dataType = lb::BRDF_DATA;
    }
    else if (data_->getBtdf()) {
        brdf = data_->getBtdf()->getBrdf();
        dataType = lb::BTDF_DATA;
    }
    else {
        return;
    }

    bxdfMeshGeode_->getOrCreateStateSet()->removeAttribute(osg::StateAttribute::POLYGONOFFSET);

    switch (displayMode_) {
        case PHOTOMETRY_DISPLAY: {
            setupBrdfMeshGeometry(brdf, inTheta_, inPhi_, wavelengthIndex, dataType, true);
            oitUsed_ = false;
            break;
        }
        case NORMAL_DISPLAY: {
            setupBrdfMeshGeometry(brdf, inTheta_, inPhi_, wavelengthIndex, dataType);
            oitUsed_ = false;
            break;
        }
        case ALL_INCOMING_POLAR_ANGLES_DISPLAY: {
            clearInDirLine();

            float curInTheta;
            #pragma omp parallel for private(curInTheta)
            for (int i = 0; i < data_->getNumInTheta(); ++i) {
                curInTheta = data_->getIncomingPolarAngle(i);
                setupBrdfMeshGeometry(brdf, curInTheta, inPhi_, wavelengthIndex, dataType);
            }

            for (int i = 0; i < data_->getNumInTheta(); ++i) {
                curInTheta = data_->getIncomingPolarAngle(i);
                float inThetaRatio = curInTheta / lb::PI_2_F;
                osg::Vec4 color(scene_util::hueToRgb(inThetaRatio), 1.0);

                lb::Vec3 curInDir = lb::SphericalCoordinateSystem::toXyz(curInTheta, inPhi_);
                osg::Vec3 dir = modifyLineLength(curInDir);
                osg::Geometry* geom = scene_util::createStippledLine(osg::Vec3(), dir, color, 2.0f);
                inDirGeode_->addDrawable(geom);
            }

            oitUsed_ = true;
            break;
        }
        case ALL_INCOMING_AZIMUTHAL_ANGLES_DISPLAY: {
            clearInDirLine();

            float endInPhi = data_->getIncomingAzimuthalAngle(data_->getNumInPhi() - 1)
                           - lb::SphericalCoordinateSystem::MAX_ANGLE1;
            bool duplicate = lb::isEqual(data_->getIncomingAzimuthalAngle(0), endInPhi);
            int numInPhi = duplicate ? data_->getNumInPhi() - 1 : data_->getNumInPhi();
            float curInPhi;
            #pragma omp parallel for private(curInPhi)
            for (int i = 0; i < numInPhi; ++i) {
                curInPhi = data_->getIncomingAzimuthalAngle(i);
                setupBrdfMeshGeometry(brdf, inTheta_, curInPhi, wavelengthIndex, dataType);
            }

            for (int i = 0; i < numInPhi; ++i) {
                curInPhi = data_->getIncomingAzimuthalAngle(i);
                float inPhiRatio = curInPhi / (2.0f * lb::PI_F);
                osg::Vec4 color(scene_util::hueToRgb(inPhiRatio), 1.0);

                lb::Vec3 curInDir = lb::SphericalCoordinateSystem::toXyz(inTheta_, curInPhi);
                osg::Vec3 dir = modifyLineLength(curInDir);
                osg::Geometry* geom = scene_util::createStippledLine(osg::Vec3(), dir, color, 2.0f);
                inDirGeode_->addDrawable(geom);
            }

            oitUsed_ = true;
            break;
        }
        case ALL_WAVELENGTHS_DISPLAY: {
            #pragma omp parallel for
            for (int i = 0; i < data_->getNumWavelengths(); ++i) {
                setupBrdfMeshGeometry(brdf, inTheta_, inPhi_, i, dataType);
            }
            oitUsed_ = true;
            break;
        }
        case SAMPLE_POINTS_DISPLAY: {
            setupBrdfMeshGeometry(brdf, inTheta_, inPhi_, wavelengthIndex, dataType);
            bxdfMeshGeode_->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonOffset(1.0f, 1.0f),
                                                                        osg::StateAttribute::ON);
            if (!data_->isInDirDependentCoordinateSystem()) break;

            // Update point geometry.
            osg::Geometry* pointGeom = scene_util::createBrdfPointGeometry(*brdf,
                                                                           inThetaIndex, inPhiIndex, wavelengthIndex,
                                                                           logPlotUsed_, baseOfLogarithm_, dataType);
            bxdfPointGeode_->removeDrawables(0, bxdfPointGeode_->getNumDrawables());
            bxdfPointGeode_->addDrawable(pointGeom);
            oitUsed_ = false;
            break;
        }
        case SAMPLE_POINT_LABELS_DISPLAY: {
            if (!data_->isInDirDependentCoordinateSystem()) break;

            scene_util::attachBrdfTextLabels(bxdfTextGeode_.get(),
                                             *brdf,
                                             inThetaIndex, inPhiIndex, wavelengthIndex,
                                             logPlotUsed_, baseOfLogarithm_, dataType);
            oitUsed_ = false;
            break;
        }
        default: {
            assert(false);
        }
    }
}

void GraphScene::setupBrdfMeshGeometry(lb::Brdf* brdf, float inTheta, float inPhi, int wavelengthIndex,
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
    if (dynamic_cast<lb::SphericalCoordinatesBrdf*>(brdf)) {
        meshGeom = scene_util::createBrdfMeshGeometry<lb::SphericalCoordinateSystem>(
            *brdf, inTheta, inPhi, wavelengthIndex,
            logPlotUsed_, baseOfLogarithm_, dataType, photometric,
            numTheta, numPhi);
    }
    else if (dynamic_cast<lb::SpecularCoordinatesBrdf*>(brdf)) {
        meshGeom = scene_util::createBrdfMeshGeometry<lb::SpecularCoordinateSystem>(
            *brdf, inTheta, inPhi, wavelengthIndex,
            logPlotUsed_, baseOfLogarithm_, dataType, photometric,
            numTheta, numPhi);
    }
    else {
        meshGeom = scene_util::createBrdfMeshGeometry<SpecularCenteredCoordinateSystem>(
            *brdf, inTheta, inPhi, wavelengthIndex,
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
        float inPhiRatio = inPhi / (2.0f * lb::PI_F);
        color = osg::Vec4(scene_util::hueToRgb(inPhiRatio), alpha);
        colorUpdate = true;
    }
    else if (displayMode_ == ALL_WAVELENGTHS_DISPLAY &&
             data_->getNumWavelengths() > 1) {
        const lb::SampleSet* ss = brdf->getSampleSet();

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

    #pragma omp critical
    bxdfMeshGeode_->addDrawable(meshGeom);
}

void GraphScene::updateSpecularReflectanceGeometry(int inThetaIndex, int inPhiIndex, int wavelengthIndex)
{
    const lb::SampleSet2D* ss2;
    if (data_->getSpecularReflectances()) {
        ss2 = data_->getSpecularReflectances();
    }
    else if (data_->getSpecularTransmittances()) {
        ss2 = data_->getSpecularTransmittances();
    }
    else {
        return;
    }

    bool paramValid = (inThetaIndex < ss2->getNumTheta() &&
                       inPhiIndex   < ss2->getNumPhi() &&
                       wavelengthIndex < ss2->getNumWavelengths());
    if (!paramValid) return;

    if (inThetaIndex != -1) {
        inTheta_ = ss2->getTheta(inThetaIndex);
    }

    if (inPhiIndex != -1) {
        inPhi_ = ss2->getPhi(inPhiIndex);
    }

    if (specularReflectanceGeode_.valid()) {
        bsdfGroup_->removeChild(specularReflectanceGeode_.get());
    }

    specularReflectanceGeode_ = new osg::Geode;
    specularReflectanceGeode_->setName("specularReflectanceGeode_");
    specularReflectanceGeode_->setNodeMask(SPECULAR_REFLECTANCE_MASK);
    bsdfGroup_->addChild(specularReflectanceGeode_.get());

    // Update the line of incoming direction.
    lb::Vec3 inDir = lb::SphericalCoordinateSystem::toXyz(inTheta_, inPhi_);
    updateInDirLine(inDir);

    const lb::Spectrum& sp = ss2->getSpectrum(inTheta_, inPhi_);
    float value;
    if (displayMode_ == PHOTOMETRY_DISPLAY) {
        value = scene_util::spectrumToY(sp, ss2->getColorModel(), ss2->getWavelengths());
    }
    else {
        value = sp[wavelengthIndex];
    }

    // Update specular reflectance.
    lb::Vec3 outDir;
    if (data_->getSpecularReflectances()) {
        outDir = lb::reflect(inDir, lb::Vec3(0.0, 0.0, 1.0));
    }
    else {
        outDir = -inDir;
    }
    lb::Vec3 outPos = outDir * value;
    osg::Vec3Array* vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3());
    vertices->push_back(osg::Vec3(outPos[0], outPos[1], outPos[2]));
    osg::Geometry* spRefGeometry = new osg::Geometry;
    spRefGeometry->setVertexArray(vertices);
    specularReflectanceGeode_->addDrawable(spRefGeometry);

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
    depth->setRange(0.0, 0.999999);
    stateSet->setAttributeAndModes(depth, osg::StateAttribute::ON);

    osg::LineWidth* lineWidth = new osg::LineWidth;
    lineWidth->setWidth(4.0f);
    stateSet->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);
}

void GraphScene::clearGraphGeometry()
{
    if (bxdfMeshGeode_.valid()) {
        bxdfMeshGeode_->removeDrawables(0, bxdfMeshGeode_->getNumDrawables());
    }

    if (bxdfPointGeode_.valid()) {
        bxdfPointGeode_->removeDrawables(0, bxdfPointGeode_->getNumDrawables());
    }

    if (bxdfTextGeode_.valid()) {
        bxdfTextGeode_->removeDrawables(0, bxdfTextGeode_->getNumDrawables());
    }

    if (specularReflectanceGeode_.valid()) {
        bsdfGroup_->removeChild(specularReflectanceGeode_.get());
    }
}
