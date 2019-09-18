// =================================================================== //
// Copyright (C) 2016-2019 Kimura Ryo                                  //
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
#include <libbsdf/Brdf/SpecularCoordinatesBrdf.h>

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

    lb::Brdf* getBrdf() { return brdf_.get(); }
    void setBrdf(lb::Brdf* brdf);

    lb::Btdf* getBtdf() { return btdf_.get(); }
    void setBtdf(lb::Btdf* btdf);

    lb::SampleSet2D* getSpecularReflectances() { return specularReflectances_; }
    void setSpecularReflectances(lb::SampleSet2D* reflectances);

    lb::SampleSet2D* getSpecularTransmittances() { return specularTransmittances_; }
    void setSpecularTransmittances(lb::SampleSet2D* transmittances);

    lb::FileType getFileType() const { return fileType_; }
    void setFileType(lb::FileType type) { fileType_ = type; }

    lb::SampleSet2D* getReflectances() { return reflectances_; }

    inline int getNumInTheta() const { return numInTheta_; }
    inline int getNumInPhi()   const { return numInPhi_; }

    inline int getNumWavelengths() const { return numWavelengths_; }

    void clearData();

    void updateBrdf();
    void updateBtdf();

    bool isEmpty() const;

    float getIncomingPolarAngle(int index) const;
    float getIncomingAzimuthalAngle(int index) const;

    float getWavelength(int index) const;

    /*! Gets the color model of used data. */
    lb::ColorModel getColorModel() const;

    lb::Vec3 getInDir(int inThetaIndex, int inPhiIndex);

    inline lb::Spectrum& getMaxValuesPerWavelength() { return maxPerWavelength_; }

    const lb::SampleSet* getSampleSet() const;

    /*! Returns true if a coordinate system has the angles of an incoming direction. */
    bool isInDirDependentCoordinateSystem() const;

    bool computedReflectances() { return reflectancesComputed_; }

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
    lb::Brdf* origBrdf_;

    lb::SampleSet2D* specularReflectances_;     /*!< The array of specular reflectance. */
    lb::SampleSet2D* specularTransmittances_;   /*!< The array of specular transmittance. */

    lb::FileType fileType_; /*! File format. */

    lb::SampleSet2D* reflectances_; /*!< Reflectances at each incoming direction. */

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
