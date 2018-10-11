// =================================================================== //
// Copyright (C) 2014-2018 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef GRAPH_SCENE_H
#define GRAPH_SCENE_H

#include <osg/Camera>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osgGA/CameraManipulator>

#include <libbsdf/Brdf/Brdf.h>
#include <libbsdf/Common/Vector.h>

#include "MaterialData.h"
#include "SceneUtil.h"

/*!
 * \class   GraphScene
 * \brief   The GraphScene class provides the scene data for 3D graph.
 */
class GraphScene
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GraphScene();
    ~GraphScene() {}

    enum DisplayMode {
        PHOTOMETRY_DISPLAY,
        NORMAL_DISPLAY,
        ALL_INCOMING_POLAR_ANGLES_DISPLAY,
        ALL_INCOMING_AZIMUTHAL_ANGLES_DISPLAY,
        ALL_WAVELENGTHS_DISPLAY,
        SAMPLE_POINTS_DISPLAY,
        SAMPLE_POINT_LABELS_DISPLAY
    };

    osg::Group* getRoot() { return root_.get(); }

    osg::Group* getBsdfGroup() { return bsdfGroup_.get(); }

    void updateView(int width, int height);

    /*! Creates and adds graph geometry to the scene. */
    void createBrdfGeode();

    void setScaleLength1(float length) { scaleLength1_ = length; }
    void setScaleLength2(float length) { scaleLength2_ = length; }
    void createAxisAndScale();
    void showScaleInPlaneOfIncidence(bool on);
    void updateScaleInPlaneOfIncidence();

    void updateGraphGeometry(int inThetaIndex, int inPhiIndex, int wavelengthIndex);
    void updateGraphGeometry(float inTheta, float inPhi, int wavelengthIndex);
    void updateGraphGeometry();

    void updateInOutDirLine(const lb::Vec3& inDir, const lb::Vec3& outDir, int wavelengthIndex);
    void updateInOutDirLine();

    void setMaterialData(MaterialData* data) { data_ = data; }

    void useOit(bool on) { oitUsed_ = on; }

    void useLogPlot(bool on) { logPlotUsed_ = on; }
    bool isLogPlot() const { return logPlotUsed_; }
    void setBaseOfLogarithm(float base) { baseOfLogarithm_ = base; }
    bool isLogPlotAcceptable();

    DisplayMode getDisplayMode() const { return displayMode_; }
    void setDisplayMode(DisplayMode mode) { displayMode_ = mode; }

    osg::Camera* getCamera() { return camera_; }
    void setCamera(osg::Camera* camera);

    osgGA::CameraManipulator* getCameraManipulator() { return cameraManipulator_; }
    void setCameraManipulator(osgGA::CameraManipulator* manipulator) { cameraManipulator_ = manipulator; }

    float getInTheta() const { return inTheta_; }
    float getInPhi() const { return inPhi_; }

private:
    GraphScene(const GraphScene&);
    GraphScene& operator=(const GraphScene&);

    /*! Attachs the shader of 3D graph to a node. */
    void attachGraphShader(osg::Node* node);

    /*! Attachs the color shader to a node. */
    void attachColorShader(osg::Node* node);

    /*! Creates a post processing group. */
    osg::Group* createPostProcessing(osg::Node* subgraph, int width, int height, int numFboSamples = 4);

    void initializeInDirLine();
    void updateInDirLine(const lb::Vec3& inDir, int wavelengthIndex);
    void clearInDirLine() { inDirGeode_->removeDrawables(0, inDirGeode_->getNumDrawables()); }

    void initializeInOutDirLine();

    /*! Modifies the length of a line from the origin concidering BRDF/BTDF. */
    osg::Vec3 modifyLineLength(const lb::Vec3& pos, int wavelengthIndex);

    void updateBrdfGeometry(int inThetaIndex, int inPhiIndex, int wavelengthIndex);

    void setupBrdfMeshGeometry(lb::Brdf* brdf, float inTheta, float inPhi, int wavelengthIndex,
                               lb::DataType dataType, bool photometric = false);

    void updateSpecularReflectanceGeometry(int inThetaIndex, int inPhiIndex, int wavelengthIndex);

    void clearGraphGeometry();

    MaterialData* data_;

    osg::ref_ptr<osg::Group> root_;

    osg::ref_ptr<osg::Group> postProcessingGroup_;
    osg::ref_ptr<osg::Group> postProcessingChild_;

    osg::ref_ptr<osg::Group> oitGroup_;
    osg::ref_ptr<osg::Group> oitChild_;

    osg::ref_ptr<osg::Group> scene_;            /*!< The group for rendered geometry. */
    osg::ref_ptr<osg::Group> accessoryGroup_;   /*!< The group for accessories like axes or text labels. */

    osg::ref_ptr<osg::Group> bsdfGroup_;
    osg::ref_ptr<osg::Geode> bxdfMeshGeode_;
    osg::ref_ptr<osg::Geode> bxdfPointGeode_;
    osg::ref_ptr<osg::Geode> bxdfTextGeode_;
    osg::ref_ptr<osg::Geode> specularReflectanceGeode_;

    osg::ref_ptr<osg::Geode> axisGeode_;
    osg::ref_ptr<osg::Geode> circleGeode_;
    osg::ref_ptr<osg::Geode> scaleInPlaneOfIncidenceGeode_;
    osg::ref_ptr<osg::Geode> axisTextLabelGeode_;
    
    osg::ref_ptr<osg::Geode> inDirGeode_;
    osg::ref_ptr<osg::Geode> inOutDirGeode_;

    osg::Camera*                camera_;
    osgGA::CameraManipulator*   cameraManipulator_;

    int     numMultiSamples_;
    bool    oitUsed_;
    int     numOitPasses_;

    bool    logPlotUsed_;
    float   baseOfLogarithm_;

    float   scaleLength1_;
    float   scaleLength2_;

    DisplayMode displayMode_;

    int inThetaIndex_;
    int inPhiIndex_;
    int wavelengthIndex_;

    /*! Incoming polar angle. This attribute is valid when an angle index (\a inThetaIndex_) is not used. */
    float inTheta_;

    /*!< Incoming azimuthal angle. This attribute is valid when an angle index (\a inPhiIndex_) is not used. */
    float inPhi_;

    lb::Vec3 pickedInDir_, pickedOutDir_;
};

#endif // GRAPH_SCENE_H
