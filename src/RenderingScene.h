// =================================================================== //
// Copyright (C) 2014-2018 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef RENDERING_SCENE_H
#define RENDERING_SCENE_H

#include <osg/Camera>
#include <osg/Group>
#include <osgGA/CameraManipulator>

#include <libbsdf/Brdf/Brdf.h>
#include <libbsdf/Brdf/SampleSet2D.h>

#include "RenderingDrawCallback.h"

/*!
 * \class   RenderingScene
 * \brief   The RenderingScene class provides the scene data for a rendering view.
 */
class RenderingScene
{
public:
    RenderingScene();

    osg::Group* getRoot() { return root_.get(); }

    osg::Group* getScene() { return scene_.get(); }

    void updateView(int width, int height);

    inline osg::Camera* getCamera() { return camera_; }
    inline void setCamera(osg::Camera* camera) { camera_ = camera; }

    inline osgGA::CameraManipulator* getCameraManipulator() { return cameraManipulator_; }
    inline void setCameraManipulator(osgGA::CameraManipulator* manipulator) { cameraManipulator_ = manipulator; }

    void setData(lb::Brdf* brdf, lb::SampleSet2D* reflectances, lb::DataType dataType)
    {
        brdf_ = brdf;
        reflectances_ = reflectances;
        dataType_ = dataType;
    }

    lb::Brdf* getBrdf() { return brdf_; }
    lb::SampleSet2D* getReflectance() { return reflectances_; }
    void setReflectance(lb::SampleSet2D* reflectances) { reflectances_ = reflectances; }

    void setLightDir(const osg::Vec3& dir) { inDirUniform_->set(dir); }

    void setLightIntensity(float intensity) { lightIntensity_ = intensity; }
    void setEnvironmentIntensity(float intensity) { environmentIntensity_ = intensity; }

    lb::Vec3 getInDir(int x, int y);
    lb::Vec3 getOutDir(int x, int y);

    /*!
     * Fits the camera position to render the entire scene. The camera looks at the center of model.
     *
     * \param camera            Camera to draw a view.
     * \param cameraDirection   Direction from the origin to a camera.
     * \param upDirection       Upward direction of a camera.
     * \param node              Scene node. If this is 0, this function uses the scene of camera.
     */
    static void fitCameraPosition(osg::Camera*      camera,
                                  const osg::Vec3&  cameraDirection,
                                  const osg::Vec3&  upDirection,
                                  osg::Node*        node = 0);

private:
    RenderingScene(const RenderingScene&);
    RenderingScene& operator=(const RenderingScene&);

    /*! Attachs a shader of BRDF rendering to a node. */
    void attachRenderingShader(osg::Node* node);

    /*! Creates a post processing group. */
    osg::Group* createPostProcessing(osg::Node* subgraph, int width, int height, int numFboSamples = 0);

    osg::Group* createRenderingGroup(osg::Group*    subgraph,
                                     int            width,
                                     int            height);

    osg::ref_ptr<osg::Group> root_;

    osg::ref_ptr<osg::Group> postProcessingGroup_;
    osg::ref_ptr<osg::Group> postProcessingChild_;

    osg::ref_ptr<osg::Group> renderingGroup_;
    osg::ref_ptr<osg::Group> renderingChild_;

    osg::ref_ptr<osg::Group> scene_; /*!< The group for rendered geometry. */

    int numMultiSamples_;

    osg::Camera*                camera_;
    osgGA::CameraManipulator*   cameraManipulator_;

    osg::ref_ptr<RenderingDrawCallback> drawCallback_;

    osg::ref_ptr<osg::Uniform> inDirUniform_;

    lb::Brdf*           brdf_;
    lb::SampleSet2D*    reflectances_;
    lb::DataType        dataType_;

    float lightIntensity_;
    float environmentIntensity_;
};

#endif // RENDERING_SCENE_H
