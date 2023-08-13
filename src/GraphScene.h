// =================================================================== //
// Copyright (C) 2014-2023 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef GRAPH_SCENE_H
#define GRAPH_SCENE_H

#include <osg/Group>

#include <libbsdf/Brdf/Brdf.h>

#include "MaterialData.h"

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

    enum class ArcDisplayMode {
        HALF_DIFF   = 0,
        SPECULAR    = 1,
        SPHERICAL   = 2
    };

    osg::Group* getRoot() { return root_; }

    osg::Group* getBsdfGroup() { return bsdfGroup_; }

    void updateView(int width, int height);

    void setScaleLength1(float length) { scaleLength1_ = length; }
    void setScaleLength2(float length) { scaleLength2_ = length; }
    void createAxisAndScale();
    void showScaleInPlaneOfIncidence(bool on);
    void updateScaleInPlaneOfIncidence();

    void updateGraphGeometry(int inThetaIndex, int inPhiIndex, int wavelengthIndex);
    void updateGraphGeometry(double inTheta, double inPhi, int wavelengthIndex);
    void updateGraphGeometry();

    void updateInOutDirLine(const lb::Vec3& inDir,
                            const lb::Vec3& outDir);
    void updateInOutDirLine();
    void updateInDirLine(const lb::Vec3& inDir);
    void clearOutDirLine();

    void setMaterialData(MaterialData* data) { data_ = data; }

    void useOit(bool on) { oitUsed_ = on; }

    void useLogPlot(bool on) { logPlotUsed_ = on; }
    bool isLogPlot() const { return logPlotUsed_; }
    void setBaseOfLogarithm(float base) { baseOfLogarithm_ = base; }
    bool isLogPlotAcceptable();

    DisplayMode getDisplayMode() const { return displayMode_; }
    void setDisplayMode(DisplayMode mode) { displayMode_ = mode; }

    ArcDisplayMode getArcDisplayMode() const { return arcDisplayMode_; }
    void setArcDisplayMode(ArcDisplayMode mode) { arcDisplayMode_ = mode; }

    bool getArcDisplayUsed() const { return arcDisplayUsed_; }
    void setArcDisplayUsed(bool on) { arcDisplayUsed_ = on; }

    int getWavelengthIndex() const { return wavelengthIndex_; }

    double getInTheta() const { return lb::SphericalCoordinateSystem::toTheta(pickedInDir_); }
    double getInPhi()   const { return lb::SphericalCoordinateSystem::toPhi(pickedInDir_); }

    const lb::Vec3& getPickedInDir()  const { return pickedInDir_; }
    const lb::Vec3& getPickedOutDir() const { return pickedOutDir_; }

private:
    GraphScene(const GraphScene&);
    GraphScene& operator=(const GraphScene&);

    /*! Creates and adds graph geometries to the scene. */
    void setupBrdfGeode();

    /*! Attaches the shader of 3D graph to a node. */
    void attachGraphShader(osg::Node* node);

    /*! Attaches the color shader to a node. */
    void attachColorShader(osg::Node* node);

    /*! Creates a post processing group. */
    osg::Group* createPostProcessing(osg::Node* subgraph, int width, int height, int numFboSamples = 4);

    /*! Modifies the length of a line from the origin considering BRDF/BTDF. */
    osg::Vec3 modifyLineLength(const lb::Vec3& pos);

    /*! Sets up the angles of a half difference coordinate system. */
    void setupHalfDiffCoordAngles(const lb::Vec3& inDir, const lb::Vec3& outDir, float arcRadius);

    /*! Sets up the angles of a specular coordinate system. */
    void setupSpecularCoordAngles(const lb::Vec3& inDir, const lb::Vec3& outDir, float arcRadius);

    /*! Sets up the angles of a spherical coordinate system. */
    void setupSphericalCoordAngles(const lb::Vec3& inDir, const lb::Vec3& outDir, float arcRadius);

    void updateBrdfGeometry(int inThetaIndex, int inPhiIndex, int wavelengthIndex);

    void setupBrdfMeshGeometry(const lb::Brdf& brdf, float inTheta, float inPhi, int wavelengthIndex,
                               lb::DataType dataType, bool photometric = false);

    void updateSpecularReflectanceGeometry(int inThetaIndex, int inPhiIndex, int wavelengthIndex);

    void clearGraphGeometry();

    MaterialData* data_;

    osg::Group* root_;

    osg::Group* postProcessingGroup_;
    osg::Group* postProcessingChild_;

    osg::Group* oitGroup_;
    osg::Group* oitChild_;

    osg::Group* scene_; /*!< The group for rendered geometry. */

    osg::Group* accessoryGroup_; /*!< The group for accessories like axes or text labels. */

    // Children of accessoryGroup_.
    osg::Group* axisGroup_;
    osg::Group* axisTextLabelGroup_;
    osg::Group* circleGroup_;
    osg::Group* inOutDirGroup_;
    osg::Group* scaleInPlaneOfIncidenceGroup_;

    osg::Group* bsdfGroup_;

    // Children of bsdfGroup_.
    osg::Group* brdfMeshGroup_;
    osg::Group* brdfPointGroup_;
    osg::Group* brdfTextGroup_;
    osg::Group* specularReflectanceGroup_;

    int     numMultiSamples_;
    bool    oitUsed_;
    int     numOitPasses_;

    bool    logPlotUsed_;
    float   baseOfLogarithm_;

    float scaleLength1_;
    float scaleLength2_;

    bool scaleInPlaneOfIncidenceUsed_;

    DisplayMode displayMode_;

    ArcDisplayMode  arcDisplayMode_;
    bool            arcDisplayUsed_;

    int inThetaIndex_;
    int inPhiIndex_;
    int wavelengthIndex_;

    lb::Vec3 pickedInDir_, pickedOutDir_;
};

#endif // GRAPH_SCENE_H
