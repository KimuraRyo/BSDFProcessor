// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
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
#include <libbsdf/Brdf/Btdf.h>
#include <libbsdf/Brdf/Integrator.h>
#include <libbsdf/Brdf/SampleSet2D.h>
#include <libbsdf/Common/Vector.h>

#include "SceneUtil.h"

/*!
 * \class   GraphScene
 * \brief   The GraphScene class provides the scene data for 3D graph.
 */
class GraphScene
{
public:
    GraphScene();
    ~GraphScene();

    enum DisplayMode {
        NORMAL_DISPLAY,
        ALL_INCOMING_POLAR_ANGLES_DISPLAY,
        ALL_WAVELENGTHS_DISPLAY,
        SAMPLE_POINTS_DISPLAY,
        SAMPLE_POINT_LABELS_DISPLAY
    };

    osg::Group* getRoot() { return root_.get(); }

    osg::Group* getBsdfGroup() { return bsdfGroup_.get(); }

    void updateView(int width, int height);

    /*! Creates and adds graph geometry to the scene. */
    void createBrdfGeode();

    void createAxis(bool useTextLabel = false, bool useLogPlot = false, float baseOfLogarithm = 10.0f);

    void updateGraphGeometry(int inThetaIndex, int inPhiIndex, int spectrumIndex);

    void clearData();

    float getIncomingPolarAngle(int index) const;

    lb::Vec3 getInDir(int inThetaIndex, int inPhiIndex);

    float getWavelength(int index) const;

    lb::Brdf* getBrdf() { return brdf_; }
    void setBrdf(lb::Brdf* brdf);

    lb::Btdf* getBtdf() { return btdf_; }
    void setBtdf(lb::Btdf* btdf);

    lb::SampleSet2D* getSpecularReflectances() { return specularReflectances_; }
    void setSpecularReflectances(lb::SampleSet2D* reflectances);

    lb::SampleSet2D* getSpecularTransmittances() { return specularTransmittances_; }
    void setSpecularTransmittances(lb::SampleSet2D* transmittances);

    lb::SampleSet2D* getReflectances() { return reflectances_; }

    inline int getNumInTheta() { return numInTheta_; }
    inline int getNumInPhi()   { return numInPhi_; }

    inline int getNumWavelengths() { return numWavelengths_; }

    /*! Gets the color model of used data. */
    lb::ColorModel getColorModel() const;

    inline lb::Spectrum& getMaxValuesPerWavelength() { return maxPerWavelength_; }

    void useOit(bool on) { useOit_ = on; }

    void useLogPlot(bool on) { useLogPlot_ = on; }
    void setBaseOfLogarithm(float base) { baseOfLogarithm_ = base; }

    void setDisplayMode(DisplayMode mode) { displayMode_ = mode; }

    inline osg::Camera* getCamera() { return camera_; }
    inline void setCamera(osg::Camera* camera) { camera_ = camera; }

    inline osgGA::CameraManipulator* getCameraManipulator() { return cameraManipulator_; }
    inline void setCameraManipulator(osgGA::CameraManipulator* manipulator) { cameraManipulator_ = manipulator; }

    const lb::SampleSet* getSampleSet() const;

    /*! Returns true if a coordinate system has the angles of an incoming direction. */
    bool isInDirDependentCoordinateSystem() const;

private:
    GraphScene(const GraphScene&);
    GraphScene& operator=(const GraphScene&);

    /*! Attachs the shader of 3D graph to a node. */
    void attachGraphShader(osg::Node* node);

    /*! Attachs the color shader to a node. */
    void attachColorShader(osg::Node* node);

    /*! Creates a post processing group. */
    osg::Group* createPostProcessing(osg::Node* subgraph, int width, int height, int numFboSamples = 4);

    void createInDirGeode();

    void updateIncomingDirectionGeometry(const lb::Vec3& inDirection);

    void updateBrdfGeometry(int inThetaIndex, int inPhiIndex, int spectrumIndex);
    void setupBrdfMeshGeometry(lb::Brdf* brdf, float inTheta, float inPhi, int spectrumIndex,
                               lb::DataType dataType);

    void updateSpecularReflectanceGeometry(int inThetaIndex, int inPhiIndex, int spectrumIndex);

    void clearGraphGeometry();

    lb::Spectrum findMaxPerWavelength(const lb::SampleSet& samples);

    void computeReflectances();

    lb::Brdf*           brdf_;
    lb::Btdf*           btdf_;
    lb::SampleSet2D*    specularReflectances_;
    lb::SampleSet2D*    specularTransmittances_;

    lb::SampleSet2D* reflectances_; /*!< Reflectance samples at each incoming direction. */

    lb::Spectrum maxPerWavelength_;

    int numInTheta_;
    int numInPhi_;

    int numWavelengths_;

    lb::Integrator* integrator_;

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
    osg::ref_ptr<osg::Geode> axisTextLabelGeode_;
    
    osg::ref_ptr<osg::Geode>    inDirGeode_;
    osg::ref_ptr<osg::Geometry> inDirGeometry_;

    int     numMultiSamples_;
    bool    useOit_;
    int     numOitPasses_;

    bool    useLogPlot_;
    float   baseOfLogarithm_;

    DisplayMode displayMode_;

    osg::Camera*                camera_;
    osgGA::CameraManipulator*   cameraManipulator_;

    /*!
     * The number of incoming directions. This is used if a coordinate system doesn't have
     * the angles of an incoming direction.
     */
    static const int NUM_INCOMING_POLAR_ANGLES = 19;
};

#endif // GRAPH_SCENE_H
