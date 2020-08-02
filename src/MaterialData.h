// =================================================================== //
// Copyright (C) 2016-2020 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef MATERIAL_DATA_H
#define MATERIAL_DATA_H

#include <memory>

#include <QObject>

#include <libbsdf/Brdf/Brdf.h>
#include <libbsdf/Brdf/Btdf.h>
#include <libbsdf/Brdf/SampleSet2D.h>

/*!
 * \class   MaterialData
 * \brief   The MaterialData class provides the BRDF/BTDF and specular reflectance/transmittance and related data.
 */
class MaterialData : public QObject
{
    Q_OBJECT

public:
    MaterialData();
    ~MaterialData();

    std::shared_ptr<lb::Brdf> getBrdf() { return brdf_; }
    void setBrdf(std::shared_ptr<lb::Brdf> brdf);

    std::shared_ptr<lb::Btdf> getBtdf() { return btdf_; }
    void setBtdf(std::shared_ptr<lb::Btdf> btdf);

    std::shared_ptr<lb::SampleSet2D> getSpecularReflectances() { return specularReflectances_; }
    void setSpecularReflectances(std::shared_ptr<lb::SampleSet2D>reflectances);

    std::shared_ptr<lb::SampleSet2D> getSpecularTransmittances() { return specularTransmittances_; }
    void setSpecularTransmittances(std::shared_ptr<lb::SampleSet2D> transmittances);

    lb::FileType getFileType() const { return fileType_; }
    void setFileType(lb::FileType type) { fileType_ = type; }

    lb::SampleSet2D*       getReflectances()       { return reflectances_.get(); }
    const lb::SampleSet2D* getReflectances() const { return reflectances_.get(); }

    inline int getNumInTheta() const { return numInTheta_; }
    inline int getNumInPhi()   const { return numInPhi_; }

    inline int getNumWavelengths() const { return numWavelengths_; }

    void clearData();

    void updateBrdf();
    void updateBtdf();

    bool isEmpty() const;
    bool isIsotropic() const;

    float getIncomingPolarAngle(int index) const;
    float getIncomingAzimuthalAngle(int index) const;

    void getHalfDiffCoordAngles(const lb::Vec3& inDir, const lb::Vec3& outDir,
                                float* halfTheta, float* halfPhi, float* diffTheta, float* diffPhi);

    void getSpecularCoordAngles(const lb::Vec3& inDir, const lb::Vec3& outDir,
                                float* inTheta, float* inPhi, float* specTheta, float* specPhi);

    void getShericalCoordAngles(const lb::Vec3& inDir, const lb::Vec3& outDir,
                                float* inTheta, float* inPhi, float* outTheta, float* outPhi);

    float getWavelength(int index) const;

    lb::Arrayf getWavelengths() const;

    /*! Gets the color model of used data. */
    lb::ColorModel getColorModel() const;

    /*! Gets the source type of used data. */
    lb::SourceType getSourceType() const;

    lb::Vec3 getInDir(int inThetaIndex, int inPhiIndex);

    inline lb::Spectrum& getMaxValuesPerWavelength() { return maxPerWavelength_; }

    /*! Gets lb::Brdf in brdf_ or btdf_. If not found, nullptr is returned. */
    lb::Brdf* getBrdfData() const;

    /*! Gets lb::SampleSet in brdf_ or btdf_. If not found, nullptr is returned. */
    lb::SampleSet* getSampleSet() const;

    /*! Gets lb::SampleSet2D from specularReflectances_ or specularTransmittances_. If not found, nullptr is returned. */
    lb::SampleSet2D* getSampleSet2D() const;

    /*! Returns true if a coordinate system has the angles of an incoming direction. */
    bool isInDirDependentCoordinateSystem() const;

    lb::DataType getDataType() const;

    bool isReflectancesComputed() { return reflectancesComputed_; }

    void editBrdf(lb::Spectrum::Scalar  glossyIntensity,
                  lb::Spectrum::Scalar  glossyShininess,
                  lb::Spectrum::Scalar  diffuseIntensity);

public slots:
    void handleReflectances();

signals:
    void computed();
    void stopReflectanceCalculator();

private:
    static lb::Spectrum findMaxPerWavelength(const lb::SampleSet& samples);

    void clearComputedData();

    void updateSampleSet(lb::SampleSet* ss);

    void computeReflectances();

    std::shared_ptr<lb::Brdf> brdf_;
    std::shared_ptr<lb::Btdf> btdf_;
    std::unique_ptr<lb::Brdf> origBrdf_;

    std::shared_ptr<lb::SampleSet2D> specularReflectances_;     /*!< The array of specular reflectance. */
    std::shared_ptr<lb::SampleSet2D> specularTransmittances_;   /*!< The array of specular transmittance. */

    std::shared_ptr<lb::SampleSet2D> reflectances_; /*!< Reflectances at each incoming direction. */

    lb::FileType fileType_; /*! File format. */

    lb::Spectrum maxPerWavelength_; /*!< Maximum values at each wavelength. */

    lb::Spectrum diffuseThresholds_; /*!< Thresholds to separate the diffuse component from a BRDF. */

    int numInTheta_;
    int numInPhi_;

    int numWavelengths_;

    bool reflectancesComputed_;

    /*!
     * The number of incoming directions. This is used if a coordinate system does not have
     * the angles of an incoming direction.
     */
    static const int NUM_INCOMING_POLAR_ANGLES = 19;
};

#endif // MATERIAL_DATA_H
 