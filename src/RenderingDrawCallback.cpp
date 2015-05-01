// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "RenderingDrawCallback.h"

#include <libbsdf/Common/Global.h>
#include <libbsdf/Common/SpectrumUtility.h>

RenderingDrawCallback::RenderingDrawCallback(osg::Image*        inDirImage,
                                             osg::Image*        outDirImage,
                                             lb::Brdf*          brdf,
                                             lb::SampleSet2D*   reflectances,
                                             lb::DataType       dataType,
                                             float              lightIntensity,
                                             float              environmentIntensity)
                                             : inDirImage_(inDirImage),
                                               outDirImage_(outDirImage),
                                               brdf_(brdf),
                                               reflectances_(reflectances),
                                               dataType_(dataType),
                                               lightIntensity_(lightIntensity),
                                               environmentIntensity_(environmentIntensity) {}

void RenderingDrawCallback::render() const
{
    float* inDirData = reinterpret_cast<float*>(inDirImage_->data());
    const float* outDirData = reinterpret_cast<const float*>(outDirImage_->data());

    float* inDirPtr;
    lb::Vec3 inDir, outDir;
    int numPixels = inDirImage_->s() * inDirImage_->t();
    #pragma omp parallel for private(inDirPtr, inDir, outDir)
    for (int i = 0; i < numPixels; ++i) {
        inDirPtr = &inDirData[i * 4];

        // Ignore background.
        if (inDirPtr[3] != 0.0f) continue;

        inDir = lb::Vec3(inDirPtr[0], inDirPtr[1], inDirPtr[2]);
        outDir = lb::Vec3(outDirData[i * 4], outDirData[i * 4 + 1], outDirData[i * 4 + 2]);

        inDir = inDir.cwiseProduct(lb::Vec3(2.0, 2.0, 2.0)) - lb::Vec3(1.0, 1.0, 1.0);
        outDir = outDir.cwiseProduct(lb::Vec3(2.0, 2.0, 2.0)) - lb::Vec3(1.0, 1.0, 1.0);
        outDir[2] = std::max(outDir[2], 0.0f);
        outDir.normalize();

        inDirPtr[0] = inDirPtr[1] = inDirPtr[2] = 0.0f;

        if (reflectances_ && environmentIntensity_ > 0.0f) {
            renderReflectance(outDir, inDirPtr);
        }

        if (dataType_ == lb::BTDF_DATA || dataType_ == lb::SPECULAR_TRANSMITTANCE_DATA) {
            inDir[2] = -inDir[2];
        }

        if (inDir[2] <= 0.0) continue;
        inDir.normalize();

        if (brdf_ && lightIntensity_ > 0.0f) {
            renderBrdf(inDir, outDir, inDirPtr);
        }
    }
}

void RenderingDrawCallback::renderBrdf(const lb::Vec3& inDir, const lb::Vec3& outDir, float* pixel) const
{
    lb::Spectrum sp = brdf_->getSpectrum(inDir, outDir);
    const lb::SampleSet* ss = brdf_->getSampleSet();

    lb::Vec3 rgb;
    if (ss->getColorModel() == lb::RGB_MODEL) {
        rgb = sp;
    }
    else if (ss->getColorModel() == lb::XYZ_MODEL) {
        rgb = lb::SpectrumUtility::xyzToSrgb(sp);
    }
    else if (ss->getNumWavelengths() == 1) {
        rgb[0] = rgb[1] = rgb[2] = sp[0];
    }
    else {
        const lb::SampleSet* ss = brdf_->getSampleSet();
        rgb = lb::SpectrumUtility::spectrumToSrgb(sp, ss->getWavelengths(), ss->getNumWavelengths());
    }

    rgb *= lb::PI_F * inDir[2] * lightIntensity_;
    pixel[0] += rgb[0];
    pixel[1] += rgb[1];
    pixel[2] += rgb[2];
}

void RenderingDrawCallback::renderReflectance(const lb::Vec3& outDir, float* pixel) const
{
    lb::Spectrum sp = reflectances_->getSpectrum(outDir);

    lb::Vec3 rgb;
    if (reflectances_->getColorModel() == lb::RGB_MODEL) {
        rgb = sp;
    }
    else if (reflectances_->getColorModel() == lb::XYZ_MODEL) {
        rgb = lb::SpectrumUtility::xyzToSrgb(sp);
    }
    else if (reflectances_->getNumWavelengths() == 1) {
        rgb[0] = rgb[1] = rgb[2] = sp[0];
    }
    else {
        rgb = lb::SpectrumUtility::spectrumToSrgb(sp,
                                                  reflectances_->getWavelengths(),
                                                  reflectances_->getNumWavelengths());
    }

    rgb *= environmentIntensity_;
    pixel[0] += rgb[0];
    pixel[1] += rgb[1];
    pixel[2] += rgb[2];
}
