// =================================================================== //
// Copyright (C) 2014-2016 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "GraphScene.h"

#include <iostream>

#include <osg/ClipPlane>
#include <osg/CullFace>
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
#include <libbsdf/Common/SpecularCoordinateSystem.h>
#include <libbsdf/Common/SphericalCoordinateSystem.h>

#include "SpecularCenteredCoordinateSystem.h"

GraphScene::GraphScene() : numMultiSamples_(4),
                           useOit_(false),
                           numOitPasses_(8),
                           useLogPlot_(false),
                           baseOfLogarithm_(10.0),
                           displayMode_(NORMAL_DISPLAY),
                           camera_(0),
                           cameraManipulator_(0)
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

    createAxis(true);
    initializeInDirLine();
    initializeInOutDirLine();
}

void GraphScene::updateView(int width, int height)
{
    assert(postProcessingGroup_.valid() && postProcessingChild_.valid());
    assert(oitGroup_.valid() && oitChild_.valid());

    int numMultiSamples = useOit_ ? 0 : numMultiSamples_;
    int numOitPasses = useOit_ ? numOitPasses_ : 1;

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

void GraphScene::createAxis(bool useTextLabel, bool useLogPlot, float baseOfLogarithm)
{
    // Add axis.
    {
        if (axisGeode_.valid()) {
            accessoryGroup_->removeChild(axisGeode_.get());
        }

        axisGeode_ = scene_util::createAxis(10.0);
        axisGeode_->setName("axisGeode_");
        attachColorShader(axisGeode_);
        accessoryGroup_->addChild(axisGeode_);
    }

    std::vector<float> radii;
    radii.push_back(0.1f);
    radii.push_back(0.2f);
    radii.push_back(0.3f);
    radii.push_back(0.4f);
    radii.push_back(0.5f);
    radii.push_back(0.6f);
    radii.push_back(0.7f);
    radii.push_back(0.8f);
    radii.push_back(0.9f);
    radii.push_back(1.0f);

    if (useLogPlot) {
        for (auto it = radii.begin(); it != radii.end(); ++it) {
            *it = scene_util::toLogValue(*it, baseOfLogarithm);
        }
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

        //circleGeode_->addDrawable(scene_util::createCircleFloor(radii.at(0), 256, 1.0f, true));
        //circleGeode_->addDrawable(scene_util::createCircleFloor(radii.at(1), 256, 1.0f, true));
        //circleGeode_->addDrawable(scene_util::createCircleFloor(radii.at(2), 256, 1.0f, true));
        //circleGeode_->addDrawable(scene_util::createCircleFloor(radii.at(3), 256, 1.0f, true));
        circleGeode_->addDrawable(scene_util::createCircleFloor(radii.at(4), 256, 1.0f, false));
        //circleGeode_->addDrawable(scene_util::createCircleFloor(radii.at(5), 256, 1.0f, true));
        //circleGeode_->addDrawable(scene_util::createCircleFloor(radii.at(6), 256, 1.0f, true));
        //circleGeode_->addDrawable(scene_util::createCircleFloor(radii.at(7), 256, 1.0f, true));
        //circleGeode_->addDrawable(scene_util::createCircleFloor(radii.at(8), 256, 1.0f, true));
        circleGeode_->addDrawable(scene_util::createCircleFloor(radii.at(9), 256, 1.0f, false));
    }

    if (axisTextLabelGeode_.valid()) {
        accessoryGroup_->removeChild(axisTextLabelGeode_.get());
    }

    // Add text labels.
    if (useTextLabel) {
        axisTextLabelGeode_ = new osg::Geode;
        axisTextLabelGeode_->setName("axisTextLabelGeode_");
        axisTextLabelGeode_->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        accessoryGroup_->addChild(axisTextLabelGeode_);

        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("0.5", osg::Vec3(radii.at(4), 0.0, 0.0)));
        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("0.5", osg::Vec3(-radii.at(4), 0.0, 0.0)));
        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("0.5", osg::Vec3(0.0, radii.at(4), 0.0)));
        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("0.5", osg::Vec3(0.0, -radii.at(4), 0.0)));

        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("1.0", osg::Vec3(radii.at(9), 0.0, 0.0)));
        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("1.0", osg::Vec3(-radii.at(9), 0.0, 0.0)));
        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("1.0", osg::Vec3(0.0, radii.at(9), 0.0)));
        axisTextLabelGeode_->addDrawable(scene_util::createTextLabel("1.0", osg::Vec3(0.0, -radii.at(9), 0.0)));
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
            clipPlane->setClipPlane(0.0, 0.0, 1.0, -0.00001);
        }
        else {
            clipPlane->setClipPlane(0.0, 0.0, -1.0, -0.00001);
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
    clearGraphGeometry();

    if (data_->getBrdf() ||
        data_->getBtdf()) {
        createAxis(false, useLogPlot_, baseOfLogarithm_);
        updateBrdfGeometry(inThetaIndex, inPhiIndex, wavelengthIndex);
    }
    else if (data_->getSpecularReflectances() ||
             data_->getSpecularTransmittances()) {
        createAxis();
        updateSpecularReflectanceGeometry(inThetaIndex, inPhiIndex, wavelengthIndex);
    }
    else {
        createAxis(true, useLogPlot_, baseOfLogarithm_);
        return;
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
    node->getOrCreateStateSet()->setName("colorShader");
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
        std::cerr << "[GraphScene::createPostProcessing] Failed to create a post-processing group." << std::endl;
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

void GraphScene::updateInDirLine(const lb::Vec3& inDir, int wavelengthIndex)
{
    osg::Vec3 dir;
    if (inDir[2] >= 0.0) {
        dir = modifyDirLineLength(inDir, wavelengthIndex);
    }
    else {
        dir = osg::Vec3(0.0, 0.0, 1.0);
        std::cerr << "[GraphScene::updateInDirLine] Negative Z-component: " << inDir << std::endl;
    }

    osg::Geometry* geom = scene_util::createStippledLine(osg::Vec3(), dir,
                                                         osg::Vec4(1.0, 0.0, 0.0, 1.0));
    inDirGeode_->removeDrawables(0, inDirGeode_->getNumDrawables());
    inDirGeode_->addDrawable(geom);
}

osg::Vec3 GraphScene::modifyDirLineLength(const lb::Vec3& lineDir, int wavelengthIndex)
{
    osg::Vec3 dir(lineDir[0], lineDir[1], lineDir[2]);

    if (data_->getBrdf() || data_->getBtdf()) {
        float val = data_->getMaxValuesPerWavelength().maxCoeff();
        if (useLogPlot_){
            val = scene_util::toLogValue(val, baseOfLogarithm_);
        }

        const float coeff = 1.2f;
        if (val > 1.0f / coeff) {
            dir *= val * coeff;
        }
    }

    return dir;
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

void GraphScene::updateInOutDirLine(const lb::Vec3& inDir,
                                    const lb::Vec3& outDir,
                                    int             wavelengthIndex)
{
    inOutDirGeode_->removeDrawables(0, inOutDirGeode_->getNumDrawables());

    if (inDir.isZero() && outDir.isZero()) {
        return;
    }

    float lineWidth = 2.0f;
    GLint stippleFactor = 1;

    float angleLength = 0.5f;
    if (useLogPlot_){
        angleLength = scene_util::toLogValue(angleLength, baseOfLogarithm_);
    }

    // Update the line of an incoming direction.
    {
        osg::Vec4 color(1.0, 0.2, 0.0, 1.0);
        GLushort stipplePattern = 0x8fff;

        osg::Vec3 dir = modifyDirLineLength(inDir, wavelengthIndex);
        osg::Geometry* geom = scene_util::createStippledLine(osg::Vec3(), dir, color,
                                                             lineWidth, stippleFactor, stipplePattern);
        inOutDirGeode_->addDrawable(geom);

        osg::Vec3 anglePos0(dir);
        anglePos0.normalize();
        anglePos0 *= angleLength;

        osg::Vec3 anglePos1(dir.x(), dir.y(), 0.0);
        anglePos1.normalize();
        anglePos1 *= angleLength;

        osg::Geometry* angleGeom = scene_util::createArc(anglePos0, anglePos1, 64, color);
        inOutDirGeode_->addDrawable(angleGeom);
    }

    // Update the line of an outgoing direction.
    {
        osg::Vec3 dir= modifyDirLineLength(outDir, wavelengthIndex);
        if (data_->getBtdf()) {
            dir = -dir;
        }
        else if (data_->getSpecularTransmittances()) {
            dir.z() = -dir.z();
        }

        osg::Vec4 color(0.0, 0.2, 1.0, 1.0);
        GLushort stipplePattern = 0xff8f;
        
        osg::Geometry* geom = scene_util::createStippledLine(osg::Vec3(), dir, color,
                                                             lineWidth, stippleFactor, stipplePattern);
        inOutDirGeode_->addDrawable(geom);
        
        osg::Vec3 anglePos0(dir);
        anglePos0.normalize();
        anglePos0 *= angleLength;

        osg::Vec3 anglePos1(dir.x(), dir.y(), 0.0);
        anglePos1.normalize();
        anglePos1 *= angleLength;

        osg::Geometry* angleGeom = scene_util::createArc(anglePos0, anglePos1, 64, color);
        inOutDirGeode_->addDrawable(angleGeom);
    }
}

void GraphScene::updateBrdfGeometry(int inThetaIndex, int inPhiIndex, int wavelengthIndex)
{
    const lb::SampleSet* ss = data_->getSampleSet();
    if (!ss) return;

    bool nodeInvalid = (!bxdfMeshGeode_.valid() ||
                        !bxdfPointGeode_.valid() ||
                        !bxdfTextGeode_.valid());
    bool paramInvalid = (inThetaIndex  >= data_->getNumInTheta() ||
                         inPhiIndex    >= data_->getNumInPhi() ||
                         wavelengthIndex >= ss->getNumWavelengths());
    if (nodeInvalid || paramInvalid) return;

    // Update the geometry of incoming direction.
    float inTheta = data_->getIncomingPolarAngle(inThetaIndex);
    float inPhi = data_->getIncomingAzimuthalAngle(inPhiIndex);
    lb::Vec3 inDir = lb::SphericalCoordinateSystem::toXyz(inTheta, inPhi);
    updateInDirLine(inDir, wavelengthIndex);

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
        case NORMAL_DISPLAY: {
            setupBrdfMeshGeometry(brdf, inTheta, inPhi, wavelengthIndex, dataType);
            useOit_ = false;
            break;
        }
        case ALL_INCOMING_POLAR_ANGLES_DISPLAY: {
            clearInDirLine();

            float curInTheta;
            #pragma omp parallel for private(curInTheta)
            for (int i = 0; i < data_->getNumInTheta(); ++i) {
                curInTheta = data_->getIncomingPolarAngle(i);
                setupBrdfMeshGeometry(brdf, curInTheta, inPhi, wavelengthIndex, dataType);
            }

            for (int i = 0; i < data_->getNumInTheta(); ++i) {
                float curInTheta = data_->getIncomingPolarAngle(i);
                float inThetaRatio = curInTheta / lb::PI_2_F;
                osg::Vec4 color(scene_util::hueToRgb(inThetaRatio), 1.0);

                lb::Vec3 curInDir = lb::SphericalCoordinateSystem::toXyz(curInTheta, inPhi);
                osg::Vec3 dir = modifyDirLineLength(curInDir, wavelengthIndex);
                osg::Geometry* geom = scene_util::createStippledLine(osg::Vec3(), dir, color, 1.0f);
                inDirGeode_->addDrawable(geom);
            }

            useOit_ = true;
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
                setupBrdfMeshGeometry(brdf, inTheta, curInPhi, wavelengthIndex, dataType);
            }

            for (int i = 0; i < numInPhi; ++i) {
                float curInPhi = data_->getIncomingAzimuthalAngle(i);
                float inPhiRatio = curInPhi / (2.0f * lb::PI_F);
                osg::Vec4 color(scene_util::hueToRgb(inPhiRatio), 1.0);

                lb::Vec3 curInDir = lb::SphericalCoordinateSystem::toXyz(inTheta, curInPhi);
                osg::Vec3 dir = modifyDirLineLength(curInDir, wavelengthIndex);
                osg::Geometry* geom = scene_util::createStippledLine(osg::Vec3(), dir, color, 1.0f);
                inDirGeode_->addDrawable(geom);
            }

            useOit_ = true;
            break;
        }
        case ALL_WAVELENGTHS_DISPLAY: {
            #pragma omp parallel for
            for (int i = 0; i < data_->getNumWavelengths(); ++i) {
                setupBrdfMeshGeometry(brdf, inTheta, inPhi, i, dataType);
            }
            useOit_ = true;
            break;
        }
        case SAMPLE_POINTS_DISPLAY: {
            setupBrdfMeshGeometry(brdf, inTheta, inPhi, wavelengthIndex, dataType);
            bxdfMeshGeode_->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonOffset(1.0f, 1.0f),
                                                                        osg::StateAttribute::ON);
            if (!data_->isInDirDependentCoordinateSystem()) break;

            // Update point geometry.
            osg::Geometry* pointGeom = scene_util::createBrdfPointGeometry(*brdf,
                                                                           inThetaIndex, inPhiIndex, wavelengthIndex,
                                                                           useLogPlot_, baseOfLogarithm_, dataType);
            bxdfPointGeode_->removeDrawables(0, bxdfPointGeode_->getNumDrawables());
            bxdfPointGeode_->addDrawable(pointGeom);
            useOit_ = false;
            break;
        }
        case SAMPLE_POINT_LABELS_DISPLAY: {
            if (!data_->isInDirDependentCoordinateSystem()) break;

            scene_util::attachBrdfTextLabels(bxdfTextGeode_.get(),
                                             *brdf,
                                             inThetaIndex, inPhiIndex, wavelengthIndex,
                                             useLogPlot_, baseOfLogarithm_, dataType);
            useOit_ = false;
            break;
        }
        default: {
            assert(false);
        }
    }
}

void GraphScene::setupBrdfMeshGeometry(lb::Brdf* brdf, float inTheta, float inPhi, int wavelengthIndex,
                                       lb::DataType dataType)
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
            *brdf, inTheta, inPhi, wavelengthIndex, useLogPlot_, baseOfLogarithm_, dataType, numTheta, numPhi);
    }
    else if (dynamic_cast<lb::SpecularCoordinatesBrdf*>(brdf)) {
        meshGeom = scene_util::createBrdfMeshGeometry<lb::SpecularCoordinateSystem>(
            *brdf, inTheta, inPhi, wavelengthIndex, useLogPlot_, baseOfLogarithm_, dataType, numTheta, numPhi);
    }
    else {
        meshGeom = scene_util::createBrdfMeshGeometry<SpecularCenteredCoordinateSystem>(
            *brdf, inTheta, inPhi, wavelengthIndex, useLogPlot_, baseOfLogarithm_, dataType, numTheta, numPhi);
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

    bool paramInvalid = (inThetaIndex  >= ss2->getNumTheta() ||
                         inPhiIndex    >= ss2->getNumPhi() ||
                         wavelengthIndex >= ss2->getNumWavelengths());
    if (paramInvalid) return;

    if (specularReflectanceGeode_.valid()) {
        bsdfGroup_->removeChild(specularReflectanceGeode_.get());
    }

    specularReflectanceGeode_ = new osg::Geode;
    specularReflectanceGeode_->setName("specularReflectanceGeode_");
    specularReflectanceGeode_->setNodeMask(SPECULAR_REFLECTANCE_MASK);
    bsdfGroup_->addChild(specularReflectanceGeode_.get());

    // Update the geometry of incoming direction.
    float inTheta = ss2->getTheta(inThetaIndex);
    float inPhi   = ss2->getPhi(inPhiIndex);
    lb::Vec3 inDir = lb::SphericalCoordinateSystem::toXyz(inTheta, inPhi);
    updateInDirLine(inDir, wavelengthIndex);

    // Update specular reflectance.
    lb::Vec3 outDir;
    if (data_->getSpecularReflectances()) {
        outDir = lb::reflect(inDir, lb::Vec3(0.0, 0.0, 1.0));
    }
    else {
        outDir = -inDir;
    }
    lb::Vec3 outPos = outDir * ss2->getSpectrum(inThetaIndex, inPhiIndex)[wavelengthIndex];
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
