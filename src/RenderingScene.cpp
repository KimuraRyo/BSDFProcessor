// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "RenderingScene.h"

#include <iostream>

#include <osg/ComputeBoundsVisitor>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>

#include "RenderingDrawCallback.h"
#include "SceneUtil.h"

RenderingScene::RenderingScene() : numMultiSamples_(0),
                                   camera_(0),
                                   cameraManipulator_(0),
                                   brdf_(0),
                                   reflectances_(0),
                                   dataType_(lb::BRDF_DATA),
                                   lightIntensity_(1.0f),
                                   environmentIntensity_(0.0f)
{
    const int windowSize = 512;

    root_ = new osg::Group;
    root_->setName("root_");

    postProcessingChild_ = new osg::Group;
    postProcessingChild_->setName("postProcessingChild_");

    postProcessingGroup_ = createPostProcessing(postProcessingChild_.get(), windowSize, windowSize);
    postProcessingGroup_->setName("postProcessingGroup_");

    renderingChild_ = new osg::Group;
    renderingChild_->setName("renderingChild_");

    renderingGroup_ = createRenderingGroup(renderingChild_.get(), windowSize, windowSize);
    renderingGroup_->setName("renderingGroup_");

    scene_ = new osg::Group;
    scene_->setName("scene_");
    attachRenderingShader(scene_.get());

    root_->addChild(postProcessingGroup_.get());
    postProcessingChild_->addChild(renderingGroup_.get());
    renderingChild_->addChild(scene_.get());
}

void RenderingScene::updateView(int width, int height)
{
    {
        osg::Group* oldGroup = postProcessingGroup_.get();
        osg::Group* newGroup = createPostProcessing(postProcessingChild_.get(), width, height);
        if (!newGroup) return;

        postProcessingGroup_ = newGroup;

        osg::Node::ParentList parents = oldGroup->getParents();
        for (auto it = parents.begin(); it != parents.end(); ++it) {
            (*it)->replaceChild(oldGroup, newGroup);
        }
    }

    {
        osg::Group* oldGroup = renderingGroup_.get();
        osg::Group* newGroup = createRenderingGroup(renderingChild_.get(), width, height);
        if (!newGroup) return;

        renderingGroup_ = newGroup;

        osg::Node::ParentList parents = oldGroup->getParents();
        for (auto it = parents.begin(); it != parents.end(); ++it) {
            (*it)->replaceChild(oldGroup, newGroup);
        }
    }
}

void RenderingScene::fitCameraPosition(osg::Camera*     camera,
                                       const osg::Vec3& cameraDirection,
                                       const osg::Vec3& upDirection,
                                       osg::Node*       node)
{
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
        using std::min;
        using std::abs;

        double minNearPlaneComponent = min(min(abs(left), abs(right)), min(abs(bottom), abs(top)));
        double minFovRadian = std::atan2(minNearPlaneComponent, zNear);
        osg::BoundingSphere bs(bb);
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
    camera->setViewMatrixAsLookAt(viewDir, bb.center(), upDirection);
}

void RenderingScene::attachRenderingShader(osg::Node* node)
{
    static const char* vertexShaderSource =
    {
        "#version 120\n"
        "\n"
        "varying vec3 position;\n"
        "varying vec3 normal;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    vec4 vertex = gl_Vertex;\n"
        "    gl_Position = gl_ModelViewProjectionMatrix * vertex;\n"
        "    position = vertex.xyz;\n"
        "    normal = gl_Normal;\n"
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
        "\n"
        "// Convert to tangent space vector.\n"
        "vec3 tangentSpace(vec3 v, vec3 N, vec3 T, vec3 B)\n"
        "{\n"
        "    mat3 tanSpaceMat;\n"
        "    tanSpaceMat[0] = T;\n"
        "    tanSpaceMat[1] = B;\n"
        "    tanSpaceMat[2] = N;\n"
        "    return v * tanSpaceMat;\n"
        "}\n"
        "\n"
        "void main()\n"
        "{\n"
        "    vec3 lightDir = normalize(vec3(0.0, 0.0, 1.0));\n"
        "    vec3 viewDir = normalize(gl_ModelViewMatrixInverse[3].xyz - position);\n"
        "    vec3 N = normalize(normal);\n"
        "    vec3 c0 = cross(N, vec3(0.0, 0.0, 1.0));\n"
        "    vec3 c1 = cross(N, vec3(0.0, 1.0, 0.0));\n"
        //"    vec3 T = (length(c0) > length(c1)) ? c0 : c1;\n"
        "    vec3 T = (abs(N) != vec3(0.0, 0.0, 1.0)) ? c0 : c1;\n"
        "    T = normalize(T);\n"
        "    vec3 B = normalize(cross(T, N));\n"
        "\n"
        "    // Avoid clamping values in osg::Image.\n"
        "    vec3 tanSpaceInDir = normalize(tangentSpace(lightDir, N, T, B)) * 0.5 + 0.5;\n"
        "    vec3 tanSpaceOutDir = normalize(tangentSpace(viewDir, N, T, B)) * 0.5 + 0.5;\n"
        "    tanSpaceInDir = max(tanSpaceInDir, 0.0);\n"
        "    tanSpaceOutDir = max(tanSpaceOutDir, 0.0);\n"
        "\n"
        "    gl_FragData[0] = vec4(tanSpaceInDir, 0.0);\n"
        "    gl_FragData[1] = vec4(tanSpaceOutDir, 0.0);\n"
        "}\n"
    };
    osg::Shader* graphFragmentShader = new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource);
    graphFragmentShader->setName("graphFragmentShader");

    osg::Program* program = new osg::Program;
    program->addShader(graphVertexShader);
    program->addShader(graphFragmentShader);
    node->getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON);
}

osg::Group* RenderingScene::createPostProcessing(osg::Node* subgraph, int width, int height, int numFboSamples)
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
        std::cerr << "[RenderingScene::createPostProcessing] Failed to create a post-processing group." << std::endl;
        return 0;
    }

    osg::Uniform* renderedTexUniform = new osg::Uniform("renderedTexture", 0);
    postProcessingGroup->getOrCreateStateSet()->addUniform(renderedTexUniform);

    const float gamma = 2.2f;
    osg::Uniform* gammaUniform = new osg::Uniform("gamma", gamma);
    postProcessingGroup->getOrCreateStateSet()->addUniform(gammaUniform);

    return postProcessingGroup;
}

osg::Group* RenderingScene::createRenderingGroup(osg::Group*    subgraph,
                                                 int            width,
                                                 int            height)
{
    if (width <= 0 || height <= 0) return 0;

    osg::ref_ptr<osg::Group> renderingGroup = new osg::Group;
    renderingGroup->setName("renderingGroup");

    osg::Texture2D* texture = new osg::Texture2D;
    texture->setTextureSize(width, height);
    texture->setInternalFormat(GL_RGBA32F_ARB);
    texture->setSourceFormat(GL_RGBA);
    texture->setSourceType(GL_FLOAT);
    texture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
    texture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
    texture->setResizeNonPowerOfTwoHint(false);

    // Set up an FBO camera.
    {
        osg::Camera* fboCamera = new osg::Camera;
        fboCamera->setName("fboCamera");
        fboCamera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        fboCamera->setReferenceFrame(osg::Transform::RELATIVE_RF);
        fboCamera->setProjectionMatrix(osg::Matrixd::identity());
        fboCamera->setViewMatrix(osg::Matrixd::identity());
        fboCamera->setViewport(0, 0, width, height);
        fboCamera->setRenderOrder(osg::Camera::PRE_RENDER);
        fboCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

        osg::Image* image0 = new osg::Image;
        osg::Image* image1 = new osg::Image;
        image0->allocateImage(width, height, 1, GL_RGBA, GL_FLOAT);
        image1->allocateImage(width, height, 1, GL_RGBA, GL_FLOAT);
        fboCamera->attach(osg::Camera::COLOR_BUFFER0, image0, numMultiSamples_, 0);
        fboCamera->attach(osg::Camera::COLOR_BUFFER1, image1, numMultiSamples_, 0);

        if (brdf_ || reflectances_) {
            fboCamera->setPostDrawCallback(new RenderingDrawCallback(image0, image1,
                                                                     brdf_, reflectances_, dataType_,
                                                                     lightIntensity_, environmentIntensity_));
            texture->setImage(image0);
        }

        renderingGroup->addChild(fboCamera);
        fboCamera->addChild(subgraph);
    }

    // Set up an HUD camera.
    {
        osg::Geode* hudGeode = new osg::Geode;
        hudGeode->setName("hudGeode");

        hudGeode->setNodeMask(HUD_MASK);

        const double hudDepth = 0.0;
        osg::Geometry* hudGeom = osg::createTexturedQuadGeometry(osg::Vec3(0.0,     0.0,    hudDepth),
                                                                 osg::Vec3(width,   0.0,    hudDepth),
                                                                 osg::Vec3(0.0,     height, hudDepth));
        hudGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        hudGeom->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
        hudGeode->addDrawable(hudGeom);

        osg::Camera* hudCamera = new osg::Camera;
        hudCamera->setName("hudCamera");
        hudCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
        hudCamera->setViewMatrix(osg::Matrix::identity());
        hudCamera->setProjectionMatrixAsOrtho2D(0, width, 0, height);
        hudCamera->setRenderOrder(osg::Camera::NESTED_RENDER);
        hudCamera->addChild(hudGeode);

        renderingGroup->addChild(hudCamera);
    }

    return renderingGroup.release();
}
