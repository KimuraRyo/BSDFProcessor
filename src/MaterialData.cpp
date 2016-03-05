// =================================================================== //
// Copyright (C) 2016 Kimura Ryo                                       //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "MaterialData.h"

#include <iostream>

#include <QThread>

#include <libbsdf/Brdf/HalfDifferenceCoordinatesBrdf.h>
#include <libbsdf/Brdf/Processor.h>
#include <libbsdf/Common/PoissonDiskDistributionOnSphere.h>

#include "ReflectanceCalculator.h"

MaterialData::MaterialData() : origBrdf_(0),
                               specularReflectances_(0),
                               specularTransmittances_(0),
                               reflectances_(0),
                               numInTheta_(1),
                               numInPhi_(1),
                               numWavelengths_(1),
                               reflectancesComputed_(false)
{
    integrator_ = new lb::Integrator(lb::PoissonDiskDistributionOnSphere::NUM_SAMPLES_ON_HEMISPHERE, true);
}

MaterialData::~MaterialData()
{
    clearData();

    delete integrator_;
}

void MaterialData::setBrdf(lb::Brdf* brdf)
{
    brdf_.reset(brdf);

    if (brdf_) {
        lb::SampleSet* ss = brdf_->getSampleSet();

        ss->updateAngleAttributes();

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

void MaterialData::setBtdf(lb::Btdf* btdf)
{
    btdf_.reset(btdf);

    if (btdf_) {
        lb::SampleSet* ss = btdf_->getSampleSet();

        ss->updateAngleAttributes();

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

void MaterialData::setSpecularReflectances(lb::SampleSet2D* reflectances)
{
    specularReflectances_ = reflectances;

    if (specularReflectances_) {
        specularReflectances_->updateAngleAttributes();

        numInTheta_     = specularReflectances_->getNumTheta();
        numInPhi_       = specularReflectances_->getNumPhi();
        numWavelengths_ = specularReflectances_->getNumWavelengths();

        maxPerWavelength_.resize(0);
        diffuseThresholds_.resize(0);
    }
}

void MaterialData::setSpecularTransmittances(lb::SampleSet2D* reflectances)
{
    specularTransmittances_ = reflectances;

    if (specularTransmittances_) {
        specularTransmittances_->updateAngleAttributes();

        numInTheta_     = specularTransmittances_->getNumTheta();
        numInPhi_       = specularTransmittances_->getNumPhi();
        numWavelengths_ = specularTransmittances_->getNumWavelengths();

        maxPerWavelength_.resize(0);
        diffuseThresholds_.resize(0);
    }
}

void MaterialData::clearData()
{
    emit stopReflectanceCalculator();

    brdf_.reset();
    btdf_.reset();

    delete origBrdf_;
    origBrdf_ = 0;

    delete specularReflectances_;
    specularReflectances_ = 0;

    delete specularTransmittances_;
    specularTransmittances_ = 0;

    delete reflectances_;
    reflectances_ = 0;

    maxPerWavelength_.resize(0);
    diffuseThresholds_.resize(0);
}

float MaterialData::getIncomingPolarAngle(int index) const
{
    if (brdf_ || btdf_) {
        const lb::SampleSet* ss = getSampleSet();

        if (isInDirDependentCoordinateSystem()) {
            return ss->getAngle0(index);
        }
        else {
            if (numInTheta_ == 1) {
                return 0.0f;
            }
            else {
                return index * lb::SphericalCoordinateSystem::MAX_ANGLE0 / (numInTheta_ - 1);
            }
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

float MaterialData::getIncomingAzimuthalAngle(int index) const
{
    if (brdf_ || btdf_) {
        const lb::SampleSet* ss = getSampleSet();

        if (isInDirDependentCoordinateSystem()) {
            return ss->getAngle1(index);
        }
        else {
            if (numInPhi_ == 1) {
                return 0.0f;
            }
            else {
                return index * lb::SphericalCoordinateSystem::MAX_ANGLE1 / (numInPhi_ - 1);
            }
        }
    }
    else if (specularReflectances_) {
        return specularReflectances_->getPhi(index);
    }
    else if (specularTransmittances_) {
        return specularTransmittances_->getPhi(index);
    }
    else {
        return 0.0f;
    }
}

float MaterialData::getWavelength(int index) const
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

lb::ColorModel MaterialData::getColorModel() const
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
        return lb::SPECTRAL_MODEL;
    }
}

lb::Vec3 MaterialData::getInDir(int inThetaIndex, int inPhiIndex)
{
    const lb::SampleSet* ss = getSampleSet();
    if (!ss) return lb::Vec3::Zero();

    float inTheta = getIncomingPolarAngle(inThetaIndex);
    float inPhi = getIncomingAzimuthalAngle(inPhiIndex);
    lb::Vec3 inDir = lb::SphericalCoordinateSystem::toXyz(inTheta, inPhi);

    assert(inDir.z() >= 0.0);
    return inDir;
}

const lb::SampleSet* MaterialData::getSampleSet() const
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

bool MaterialData::isInDirDependentCoordinateSystem() const
{
    if ((brdf_ && dynamic_cast<lb::HalfDifferenceCoordinatesBrdf*>(brdf_.get())) ||
        (btdf_ && dynamic_cast<lb::HalfDifferenceCoordinatesBrdf*>(btdf_->getBrdf()))) {
        return false;
    }
    else {
        return true;
    }
}

void MaterialData::editBrdf(lb::Spectrum::Scalar    glossyIntensity,
                            lb::Spectrum::Scalar    glossyShininess,
                            lb::Spectrum::Scalar    diffuseIntensity)
{
    lb::Brdf* brdf;
    if (brdf_) {
        brdf = brdf_.get();
    } else if (btdf_) {
        brdf = btdf_->getBrdf();
    }
    else {
        return;
    }

    if (!origBrdf_) {
        origBrdf_ = brdf->clone();
        diffuseThresholds_ = findDiffuseThresholds(*origBrdf_, lb::toRadian(60.0f));
    }

    editBrdf(*origBrdf_, brdf, diffuseThresholds_,
             glossyIntensity, glossyShininess, diffuseIntensity);

    computeReflectances();

    maxPerWavelength_ = findMaxPerWavelength(*brdf->getSampleSet());
}

void MaterialData::handleReflectances()
{
    reflectancesComputed_ = true;
    emit computed();
}

lb::Spectrum MaterialData::findDiffuseThresholds(const lb::Brdf&    brdf,
                                                 float              maxPolarAngle)
{
    const lb::SampleSet* ss = brdf.getSampleSet();

    lb::Spectrum thresholds(ss->getNumWavelengths());
    thresholds.fill(std::numeric_limits<lb::Spectrum::Scalar>::max());

    for (int i0 = 0; i0 < ss->getNumAngles0(); ++i0) {
    for (int i1 = 0; i1 < ss->getNumAngles1(); ++i1) {
    for (int i2 = 0; i2 < ss->getNumAngles2(); ++i2) {
    for (int i3 = 0; i3 < ss->getNumAngles3(); ++i3) {
        lb::Vec3 inDir, outDir;
        brdf.getInOutDirection(i0, i1, i2, i3, &inDir, &outDir);

        float inTheta = std::acos(inDir[2]);
        float outTheta = std::acos(outDir[2]);

        if (inTheta <= maxPolarAngle && outTheta <= maxPolarAngle) {
            thresholds = thresholds.cwiseMin(ss->getSpectrum(i0, i1, i2, i3));
        }
    }}}}

    thresholds = thresholds.cwiseMax(0.0);

    return thresholds;
}

void MaterialData::editBrdf(const lb::Brdf&         origBrdf,
                            lb::Brdf*               brdf,
                            const lb::Spectrum&     diffuseThresholds,
                            lb::Spectrum::Scalar    glossyIntensity,
                            lb::Spectrum::Scalar    glossyShininess,
                            lb::Spectrum::Scalar    diffuseIntensity)
{
    const lb::SampleSet* ss = brdf->getSampleSet();

    for (int i0 = 0; i0 < ss->getNumAngles0(); ++i0) {
    for (int i1 = 0; i1 < ss->getNumAngles1(); ++i1) {
    for (int i2 = 0; i2 < ss->getNumAngles2(); ++i2) {
    #pragma omp parallel for
    for (int i3 = 0; i3 < ss->getNumAngles3(); ++i3) {
        editBrdf(i0, i1, i2, i3,
                 origBrdf, brdf,
                 diffuseThresholds,
                 glossyIntensity, glossyShininess, diffuseIntensity);
    }}}}
}

void MaterialData::editBrdf(int i0, int i1, int i2, int i3,
                            const lb::Brdf&         origBrdf,
                            lb::Brdf*               brdf,
                            const lb::Spectrum&     diffuseThresholds,
                            lb::Spectrum::Scalar    glossyIntensity,
                            lb::Spectrum::Scalar    glossyShininess,
                            lb::Spectrum::Scalar    diffuseIntensity)
{
    lb::SampleSet* ss = brdf->getSampleSet();

    lb::Vec3 inDir, outDir;
    brdf->getInOutDirection(i0, i1, i2, i3, &inDir, &outDir);

    // Edit a BRDF with glossy shininess.
    if (glossyShininess != 1.0) {
        float inTh, inPh, specTh, specPh;
        lb::SpecularCoordinateSystem::fromXyz(inDir, outDir, &inTh, &inPh, &specTh, &specPh);
        float specThWeight = specTh / lb::SpecularCoordinateSystem::MAX_ANGLE2;
        specThWeight = std::pow(specThWeight, 1.0f / std::max(glossyShininess, lb::EPSILON_F));
        float newSpecTh = specThWeight * lb::SpecularCoordinateSystem::MAX_ANGLE2;
        lb::SpecularCoordinateSystem::toXyz(inTh, inPh, newSpecTh, specPh, &inDir, &outDir);
        
        if (outDir[2] <= 0.0) {
            outDir[2] = 0.0;
        }
        outDir.normalize();
    }

    lb::Spectrum origSp = origBrdf.getSpectrum(inDir, outDir);
    lb::Spectrum sp(origSp.size());

    // Edit a BRDF with glossy and diffuse intensity.
    for (int i = 0; i < sp.size(); ++i) {
        lb::Spectrum::Scalar origVal = origSp[i];
        lb::Spectrum::Scalar threshold = diffuseThresholds[i];
        if (origVal <= threshold) {
            sp[i] = origVal * diffuseIntensity;
        }
        else {
            lb::Spectrum::Scalar glossy = origVal - threshold;
            sp[i] = glossy * glossyIntensity + threshold * diffuseIntensity;
        }
    }

    ss->setSpectrum(i0, i1, i2, i3, sp);
}

lb::Spectrum MaterialData::findMaxPerWavelength(const lb::SampleSet& samples)
{
    lb::Spectrum maxSp = lb::Spectrum::Zero(samples.getNumWavelengths());

    for (int i0 = 0; i0 < samples.getNumAngles0(); ++i0) {
    for (int i1 = 0; i1 < samples.getNumAngles1(); ++i1) {
    for (int i2 = 0; i2 < samples.getNumAngles2(); ++i2) {
    for (int i3 = 0; i3 < samples.getNumAngles3(); ++i3) {
        const lb::Spectrum& sp = samples.getSpectrum(i0, i1, i2, i3);
        maxSp = maxSp.cwiseMax(sp);
    }}}}

    return maxSp;
}

void MaterialData::computeReflectances()
{
    std::cout << "[MaterialData::computeReflectances]" << std::endl;

    const lb::SampleSet* ss = getSampleSet();
    if (!ss) return;

    emit stopReflectanceCalculator();

    if (!reflectances_) {
        reflectances_ = new lb::SampleSet2D(numInTheta_, numInPhi_,
                                            ss->getColorModel(),
                                            ss->getNumWavelengths());

        reflectances_->getWavelengths() = ss->getWavelengths();

        for (int inThIndex = 0; inThIndex < numInTheta_; ++inThIndex) {
            float inTheta = getIncomingPolarAngle(inThIndex);
            reflectances_->setTheta(inThIndex, inTheta);
        }

        for (int inPhIndex = 0; inPhIndex < numInPhi_; ++inPhIndex) {
            float inPhi = getIncomingAzimuthalAngle(inPhIndex);
            reflectances_->setPhi(inPhIndex, inPhi);
        }

        lb::fillSpectra(reflectances_->getSpectra(), 0.0);
    }

    reflectancesComputed_ = false;
    
    // Compute reflectance and transmittance.
    ReflectanceCalculator* calc;
    if (brdf_) {
        calc = new ReflectanceCalculator(reflectances_, brdf_, integrator_);
    }
    else if (btdf_) {
        calc = new ReflectanceCalculator(reflectances_, btdf_, integrator_);
    }
    else {
        return;
    }

    QThread* workerThread = new QThread;
    calc->moveToThread(workerThread);

    connect(workerThread, SIGNAL(started()), calc, SLOT(computeReflectances()));
    connect(this, SIGNAL(stopReflectanceCalculator()), calc, SLOT(stop()));
    connect(calc, SIGNAL(finished()), this, SLOT(handleReflectances()));
    connect(calc, SIGNAL(finished()), workerThread, SLOT(quit()));
    connect(calc, SIGNAL(stopped()), workerThread, SLOT(quit()));
    connect(workerThread, SIGNAL(finished()), calc, SLOT(deleteLater()));
    connect(workerThread, SIGNAL(finished()), workerThread, SLOT(deleteLater()));

    workerThread->start(QThread::LowestPriority);
}
