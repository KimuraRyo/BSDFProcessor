// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
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

#include <QtGui/QColor>

#include <libbsdf/Brdf/HalfDifferenceCoordinatesBrdf.h>
#include <libbsdf/Brdf/SpecularCoordinatesBrdf.h>
#include <libbsdf/Brdf/SphericalCoordinatesBrdf.h>
#include <libbsdf/Common/PoissonDiskDistributionOnSphere.h>
#include <libbsdf/Common/SpectrumUtility.h>
#include <libbsdf/Common/SphericalCoordinateSystem.h>

GraphScene::GraphScene() : brdf_(0),
                           btdf_(0),
                           specularReflectances_(0),
                           specularTransmittances_(0),
                           reflectances_(0),
                           numInTheta_(1),
                           numInPhi_(1),
                           numWavelengths_(1),
                           numMultiSamples_(4),
                           useOit_(false),
                           numOitPasses_(8),
                           useLogPlot_(false),
                           baseOfLogarithm_(10.0),
                           displayMode_(NORMAL),
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
    accessoryGroup_->setNodeMask(NodeMask::UNDEFINED);

    root_->addChild(postProcessingGroup_.get());
    postProcessingChild_->addChild(oitGroup_.get());
    oitChild_->addChild(scene_.get());
    scene_->addChild(bsdfGroup_.get());
    scene_->addChild(accessoryGroup_.get());

    createAxis(true);
    createInDirGeode();

    integrator_ = new lb::Integrator(lb::PoissonDiskDistributionOnSphere::NUM_SAMPLES_ON_HEMISPHERE, true);
}

GraphScene::~GraphScene()
{
    delete brdf_;
    delete btdf_;
    delete specularReflectances_;
    delete specularTransmittances_;
    delete reflectances_;
    delete integrator_;
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
            *it = std::log(*it + 1.0f) / std::log(baseOfLogarithm);
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
    const lb::SampleSet* ss = getSampleSet();
    if (!ss) return;

    if (bxdfMeshGeode_.valid()) {
        bsdfGroup_->removeChild(bxdfMeshGeode_.get());
    }

    // Set up mesh.
    {
        bxdfMeshGeode_ = new osg::Geode;
        bxdfMeshGeode_->setName("bxdfMeshGeode_");
        bxdfMeshGeode_->setNodeMask(NodeMask::BRDF);
        bsdfGroup_->addChild(bxdfMeshGeode_.get());

        osg::ClipPlane* clipPlane = new osg::ClipPlane;
        if (brdf_) {
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
        //bxdfPointGeode_->setNodeMask(NodeMask::BRDF);
        bsdfGroup_->addChild(bxdfPointGeode_.get());
    }

    // Set up text labels of sample points.
    {
        if (bxdfTextGeode_.valid()) {
            bsdfGroup_->removeChild(bxdfTextGeode_.get());
        }

        bxdfTextGeode_ = new osg::Geode;
        bxdfTextGeode_->setName("bxdfTextGeode_");
        //bxdfTextGeode_->setNodeMask(NodeMask::BRDF);
        bsdfGroup_->addChild(bxdfTextGeode_.get());
    }
}

void GraphScene::updateGraphGeometry(int inThetaIndex, int inPhiIndex, int spectrumIndex)
{
    clearGraphGeometry();

    if (brdf_ || btdf_) {
        createAxis(false, useLogPlot_, baseOfLogarithm_);
        updateBrdfGeometry(inThetaIndex, inPhiIndex, spectrumIndex);
    }
    else if (specularReflectances_ || specularTransmittances_) {
        createAxis();
        updateSpecularReflectanceGeometry(inThetaIndex, inPhiIndex, spectrumIndex);
    }
    else {
        createAxis(true, useLogPlot_, baseOfLogarithm_);
        return;
    }
}

void GraphScene::clearData()
{
    delete brdf_;
    delete btdf_;
    delete specularReflectances_;
    delete specularTransmittances_;

    brdf_ = 0;
    btdf_ = 0;
    specularReflectances_ = 0;
    specularTransmittances_ = 0;

    displayMode_ = NORMAL;
}

float GraphScene::getIncomingPolarAngle(int index) const
{
    if (brdf_ || btdf_) {
        const lb::SampleSet* ss = getSampleSet();

        if (isInDirDependentCoordinateSystem()) {
            return ss->getAngle0(index);
        }
        else {
            return index * lb::SphericalCoordinateSystem::MAX_ANGLE0 / (numInTheta_ - 1);
        }
    }
    else if (specularReflectances_) {
        return specularReflectances_->getTheta(index);
    }
    else if (specularTransmittances_) {
        return specularTransmittances_->getTheta(index);
    }
    else {
        return 0.0f;
    }
}

float GraphScene::getWavelength(int index) const
{
    if (brdf_ || btdf_) {
        return getSampleSet()->getWavelength(index);
    }
    else if (specularReflectances_) {
        return specularReflectances_->getWavelength(index);
    }
    else if (specularTransmittances_) {
        return specularTransmittances_->getWavelength(index);
    }
    else {
        return 0.0f;
    }
}

void GraphScene::setBrdf(lb::Brdf* brdf)
{
    brdf_ = brdf;

    if (brdf_) {
        lb::SampleSet* ss = brdf_->getSampleSet();

        ss->checkEqualIntervalAngles();

        if (isInDirDependentCoordinateSystem()) {
            numInTheta_ = ss->getNumAngles0();
        }
        else {
            numInTheta_ = NUM_INCOMING_POLAR_ANGLES;
        }

        numInPhi_ = ss->getNumAngles1();
        numWavelengths_ = ss->getNumWavelengths();

        maxPerWavelength_ = findMaxPerWavelength(*ss);
        computeReflectances();
    }
}

void GraphScene::setBtdf(lb::Btdf* btdf)
{
    btdf_ = btdf;

    if (btdf_) {
        lb::SampleSet* ss = btdf_->getSampleSet();

        ss->checkEqualIntervalAngles();

        if (isInDirDependentCoordinateSystem()) {
            numInTheta_ = ss->getNumAngles0();
        }
        else {
            numInTheta_ = NUM_INCOMING_POLAR_ANGLES;
        }

        numInPhi_ = ss->getNumAngles1();
        numWavelengths_ = ss->getNumWavelengths();

        maxPerWavelength_ = findMaxPerWavelength(*ss);
        computeReflectances();
    }
}

void GraphScene::setSpecularReflectances(lb::SampleSet2D* reflectances)
{
    specularReflectances_ = reflectances;

    if (specularReflectances_) {
        specularReflectances_->checkEqualIntervalAngles();

        numInTheta_     = specularReflectances_->getNumTheta();
        numInPhi_       = specularReflectances_->getNumPhi();
        numWavelengths_ = specularReflectances_->getNumWavelengths();

        maxPerWavelength_.resize(0);
    }
}

void GraphScene::setSpecularTransmittances(lb::SampleSet2D* reflectances)
{
    specularTransmittances_ = reflectances;

    if (specularTransmittances_) {
        specularTransmittances_->checkEqualIntervalAngles();

        numInTheta_     = specularTransmittances_->getNumTheta();
        numInPhi_       = specularTransmittances_->getNumPhi();
        numWavelengths_ = specularTransmittances_->getNumWavelengths();

        maxPerWavelength_.resize(0);
    }
}

lb::ColorModel::Type GraphScene::getColorModel() const
{
    if (brdf_ || btdf_) {
        return getSampleSet()->getColorModel();
    }
    else if (specularReflectances_) {
        return specularReflectances_->getColorModel();
    }
    else if (specularTransmittances_) {
        return specularTransmittances_->getColorModel();
    }
    else {
        return lb::ColorModel::SPECTRAL;
    }
}

const lb::SampleSet* GraphScene::getSampleSet() const
{
    const lb::SampleSet* ss;
    if (brdf_) {
        ss = brdf_->getSampleSet();
    }
    else if (btdf_) {
        ss = btdf_->getSampleSet();
    }
    else {
        return 0;
    }

    return ss;
}

bool GraphScene::isInDirDependentCoordinateSystem() const
{
    if ((brdf_ && dynamic_cast<lb::HalfDifferenceCoordinatesBrdf*>(brdf_)) ||
        (btdf_ && dynamic_cast<lb::HalfDifferenceCoordinatesBrdf*>(btdf_->getBrdf()))) {
        return false;
    }
    else {
        return true;
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
        "uniform ivec2           viewSize; // Width and height of view\n"
        "uniform bool            oitFirstPass;\n"
        "uniform sampler2DShadow oitPrevDepthTexture;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    if (!oitFirstPass) {\n"
        "        vec3 depthTexCoord = vec3(gl_FragCoord.xy / viewSize, gl_FragCoord.z);\n"
        "        if (shadow2D(oitPrevDepthTexture, depthTexCoord).x >= 0.5) {\n"
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
        "uniform ivec2           viewSize; // Width and height of view\n"
        "uniform bool            oitFirstPass;\n"
        "uniform sampler2DShadow oitPrevDepthTexture;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    if (!oitFirstPass) {\n"
        "        vec3 depthTexCoord = vec3(gl_FragCoord.xy / viewSize, gl_FragCoord.z);\n"
        "        if (shadow2D(oitPrevDepthTexture, depthTexCoord).x >= 0.5) {\n"
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

void GraphScene::createInDirGeode()
{
    if (inDirGeode_.valid()) {
        accessoryGroup_->removeChild(inDirGeode_.get());
    }
    
    inDirGeode_ = new osg::Geode;
    inDirGeode_->setName("inDirGeode_");
    attachColorShader(inDirGeode_.get());
    accessoryGroup_->addChild(inDirGeode_.get());
    
    //inDirGeometry_ = new osg::Geometry;
    inDirGeometry_ = scene_util::createInDirLine();
    inDirGeometry_->setName("inDirGeometry_");
    inDirGeode_->addDrawable(inDirGeometry_.get());
}

void GraphScene::updateIncomingDirectionGeometry(const lb::Vec3& inDirection)
{
    osg::Vec3Array* vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(0.0, 0.0, 0.0));
    if (inDirection[2] >= 0.0) {
        vertices->push_back(osg::Vec3(inDirection[0], inDirection[1], inDirection[2]));
    }
    else {
        vertices->push_back(osg::Vec3(0.0, 0.0, 1.0));
        std::cerr << "[GraphScene::updateIncomingDirectionGeometry] Negative Z-component: " << inDirection << std::endl;
    }
    inDirGeometry_->setVertexArray(vertices);
}

void GraphScene::updateBrdfGeometry(int inThetaIndex, int inPhiIndex, int spectrumIndex)
{
    const lb::SampleSet* ss = getSampleSet();
    if (!ss) return;

    bool isInvalidData = (!bxdfMeshGeode_.valid() || !bxdfPointGeode_.valid() || !bxdfTextGeode_.valid());
    bool isInvalidParam = (inThetaIndex  >= numInTheta_ ||
                           inPhiIndex    >= numInPhi_ ||
                           spectrumIndex >= ss->getNumWavelengths());
    if (isInvalidData || isInvalidParam) return;

    // Update the geometry of incoming direction.
    float inTheta = getIncomingPolarAngle(inThetaIndex);
    float inPhi = ss->getAngle1(inPhiIndex);
    lb::Vec3 inDir = lb::SphericalCoordinateSystem::toXyz(inTheta, inPhi);
    updateIncomingDirectionGeometry(inDir);

    lb::Brdf* brdf;
    bool isBtdf;
    if (brdf_) {
        brdf = brdf_;
        isBtdf = false;
    }
    else if (btdf_) {
        brdf = btdf_->getBrdf();
        isBtdf = true;
    }
    else {
        return;
    }

    bxdfMeshGeode_->getOrCreateStateSet()->removeAttribute(osg::StateAttribute::POLYGONOFFSET);

    switch (displayMode_) {
        case NORMAL: {
            setupBrdfMeshGeometry(brdf, inTheta, inPhi, spectrumIndex, isBtdf);
            useOit_ = false;
            break;
        }
        case ALL_INCOMING_POLAR_ANGLES: {
            #pragma omp parallel for
            for (int i = 0; i < numInTheta_; ++i) {
                setupBrdfMeshGeometry(brdf, getIncomingPolarAngle(i), inPhi, spectrumIndex, isBtdf);
            }

            // Remove incoming direction geometry.
            inDirGeometry_->setVertexArray(0);

            useOit_ = true;
            break;
        }
        case ALL_WAVELENGTHS: {
            #pragma omp parallel for
            for (int i = 0; i < numWavelengths_; ++i) {
                setupBrdfMeshGeometry(brdf, inTheta, inPhi, i, isBtdf);
            }
            useOit_ = true;
            break;
        }
        case SAMPLE_POINTS: {
            setupBrdfMeshGeometry(brdf, inTheta, inPhi, spectrumIndex, isBtdf);
            bxdfMeshGeode_->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonOffset(1.0f, 1.0f),
                                                                        osg::StateAttribute::ON);
            if (!isInDirDependentCoordinateSystem()) break;

            // Update point geometry.
            osg::Geometry* pointGeom = scene_util::createBrdfPointGeometry(*brdf,
                                                                           inThetaIndex, inPhiIndex, spectrumIndex,
                                                                           useLogPlot_, baseOfLogarithm_, isBtdf);
            bxdfPointGeode_->removeDrawables(0, bxdfPointGeode_->getNumDrawables());
            bxdfPointGeode_->addDrawable(pointGeom);
            useOit_ = false;
            break;
        }
        case SAMPLE_POINT_LABELS: {
            if (!isInDirDependentCoordinateSystem()) break;

            scene_util::attachBrdfTextLabels(bxdfTextGeode_.get(),
                                             *brdf,
                                             inThetaIndex, inPhiIndex, spectrumIndex,
                                             useLogPlot_, baseOfLogarithm_, isBtdf);
            useOit_ = false;
            break;
        }
        default: {
            assert(false);
        }
    }
}

void GraphScene::setupBrdfMeshGeometry(lb::Brdf* brdf, float inTheta, float inPhi, int spectrumIndex, bool isBtdf)
{
    bool isManySamples = (displayMode_ == ALL_INCOMING_POLAR_ANGLES && numInTheta_ > 10) ||
                         (displayMode_ == ALL_WAVELENGTHS && numWavelengths_ > 10);
    int numTheta, numPhi;
    if (isManySamples) {
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
            *brdf, inTheta, inPhi, spectrumIndex, useLogPlot_, baseOfLogarithm_, isBtdf, numTheta, numPhi);
    }
    else if (dynamic_cast<lb::SpecularCoordinatesBrdf*>(brdf)) {
        meshGeom = scene_util::createBrdfMeshGeometry<lb::SpecularCoordinateSystem>(
            *brdf, inTheta, inPhi, spectrumIndex, useLogPlot_, baseOfLogarithm_, isBtdf, numTheta, numPhi);
    }
    else {
        meshGeom = scene_util::createBrdfMeshGeometry<SpecularCenteredCoordinateSystem>(
            *brdf, inTheta, inPhi, spectrumIndex, useLogPlot_, baseOfLogarithm_, isBtdf, numTheta, numPhi);
    }

    // Set a color of graph.
    const float alpha = 0.3f;
    osg::Vec4 color;
    bool updateColor = false;
    if (displayMode_ == ALL_INCOMING_POLAR_ANGLES && numInTheta_ > 1) {
        float inThetaRatio = inTheta / lb::PI_2_F;
        color = osg::Vec4(scene_util::hueToRgb(inThetaRatio), alpha);
        updateColor = true;
    }
    else if (displayMode_ == ALL_WAVELENGTHS && numWavelengths_ > 1) {
        const lb::SampleSet* ss = brdf->getSampleSet();

        bool isRgb = ss->getColorModel() == lb::ColorModel::RGB;
        bool isXyz = ss->getColorModel() == lb::ColorModel::XYZ;
        if (isRgb || isXyz) {
            if (spectrumIndex == 0) {
                color = osg::Vec4(1.0, 0.0, 0.0, alpha);
            }
            else if (spectrumIndex == 1) {
                color = osg::Vec4(0.0, 1.0, 0.0, alpha);
            }
            else if (spectrumIndex == 2) {
                color = osg::Vec4(0.0, 0.0, 1.0, alpha);
            }
        }
        else {
            const lb::Arrayf& wls = ss->getWavelengths();
            lb::Vec3 rgb = lb::SpectrumUtility::wavelengthToSrgb(wls[spectrumIndex]);
            color = osg::Vec4(scene_util::toOsg(rgb), alpha);
        }
        updateColor = true;
    }

    if (updateColor) {
        osg::Vec4Array* colors = new osg::Vec4Array;
        colors->push_back(color);
        meshGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
    }

    #pragma omp critical
    bxdfMeshGeode_->addDrawable(meshGeom);
}

void GraphScene::updateSpecularReflectanceGeometry(int inThetaIndex, int inPhiIndex, int spectrumIndex)
{
    const lb::SampleSet2D* ss2;
    if (specularReflectances_) {
        ss2 = specularReflectances_;
    }
    else if (specularTransmittances_) {
        ss2 = specularTransmittances_;
    }
    else {
        return;
    }

    bool isInvalidParam = (inThetaIndex  >= ss2->getNumTheta() ||
                           inPhiIndex    >= ss2->getNumPhi() ||
                           spectrumIndex >= ss2->getNumWavelengths());
    if (isInvalidParam) return;

    if (specularReflectanceGeode_.valid()) {
        bsdfGroup_->removeChild(specularReflectanceGeode_.get());
    }

    specularReflectanceGeode_ = new osg::Geode;
    specularReflectanceGeode_->setName("specularReflectanceGeode_");
    specularReflectanceGeode_->setNodeMask(NodeMask::SPECULAR_REFLECTANCE);
    bsdfGroup_->addChild(specularReflectanceGeode_.get());

    // Update the geometry of incoming direction.
    float inTheta = ss2->getTheta(inThetaIndex);
    float inPhi   = ss2->getPhi(inPhiIndex);
    lb::Vec3 inDir = lb::SphericalCoordinateSystem::toXyz(inTheta, inPhi);
    updateIncomingDirectionGeometry(inDir);

    // Update specular reflectance.
    lb::Vec3 outDir;
    if (specularReflectances_) {
        outDir = lb::reflect(inDir, lb::Vec3(0.0, 0.0, 1.0));
    }
    else {
        outDir = -inDir;
    }
    lb::Vec3 outPos = outDir * ss2->getSpectrum(inThetaIndex, inPhiIndex)[spectrumIndex];
    osg::Vec3Array* vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(0.0, 0.0, 0.0));
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

lb::Vec3 GraphScene::getInDir(int inThetaIndex, int inPhiIndex)
{
    const lb::SampleSet* ss = getSampleSet();
    if (!ss) return lb::Vec3::Zero();

    float inTheta = getIncomingPolarAngle(inThetaIndex);
    float inPhi = ss->getAngle1(inPhiIndex);
    lb::Vec3 inDir = lb::SphericalCoordinateSystem::toXyz(inTheta, inPhi);

    assert(inDir.z() >= 0.0);
    return inDir;
}

lb::Spectrum GraphScene::findMaxPerWavelength(const lb::SampleSet& samples)
{
    lb::Spectrum maxSp;
    maxSp.resize(samples.getNumWavelengths());
    maxSp = lb::Spectrum::Zero(maxSp.size());

    for (int i0 = 0; i0 < samples.getNumAngles0(); ++i0) {
    for (int i1 = 0; i1 < samples.getNumAngles1(); ++i1) {
    for (int i2 = 0; i2 < samples.getNumAngles2(); ++i2) {
    for (int i3 = 0; i3 < samples.getNumAngles3(); ++i3) {
        const lb::Spectrum& sp = samples.getSpectrum(i0, i1, i2, i3);
        maxSp = maxSp.cwiseMax(sp);
    }}}}

    return maxSp;
}

void GraphScene::computeReflectances()
{
    osg::Timer_t startTick = osg::Timer::instance()->tick();

    const lb::SampleSet* ss = getSampleSet();
    if (!ss) return;

    const lb::Brdf* brdf;
    if (brdf_) {
        brdf = brdf_;
    }
    else if (btdf_) {
        brdf = btdf_->getBrdf();
    }
    else {
        return;
    }

    // Set up a reflectance and transmittance sample set.
    delete reflectances_;
    reflectances_ = new lb::SampleSet2D(numInTheta_, numInPhi_, ss->getColorModel(), ss->getNumWavelengths());
    reflectances_->getWavelengths() = ss->getWavelengths();
    
    for (int inThIndex = 0; inThIndex < numInTheta_; ++inThIndex) {
        lb::Vec3 inDir = getInDir(inThIndex, 0);
        float inTheta, inPhi;
        lb::SphericalCoordinateSystem::fromXyz(inDir, &inTheta, &inPhi);
        reflectances_->setTheta(inThIndex, inTheta);
    }

    for (int inPhIndex = 0; inPhIndex < numInPhi_; ++inPhIndex) {
        lb::Vec3 inDir = getInDir(0, inPhIndex);
        float inTheta, inPhi;
        lb::SphericalCoordinateSystem::fromXyz(inDir, &inTheta, &inPhi);
        reflectances_->setPhi(inPhIndex, inPhi);
    }

    // Compute reflectance and transmittance.
    for (int inPhIndex = 0; inPhIndex < numInPhi_;   ++inPhIndex) {
    for (int inThIndex = 0; inThIndex < numInTheta_; ++inThIndex) {
        lb::Vec3 inDir = getInDir(inThIndex, inPhIndex);
        lb::Spectrum sp = integrator_->computeReflectance(*brdf, inDir);
        reflectances_->setSpectrum(inThIndex, inPhIndex, sp);
    }}

    osg::Timer_t endTick = osg::Timer::instance()->tick();
    double delta = osg::Timer::instance()->delta_s(startTick, endTick);
    std::cout << "[GraphScene::computeReflectances] " << delta << "(s)" << std::endl;
}
