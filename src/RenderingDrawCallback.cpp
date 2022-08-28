// =================================================================== //
// Copyright (C) 2014-2022 Kimura Ryo                                  //
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
                                               environmentIntensity_(environmentIntensity)
{
    copiedInDirImage_ = new osg::Image(*inDirImage_);
}

void RenderingDrawCallback::render() const
{
    float* inDirData = reinterpret_cast<float*>(inDirImage_->data());
    const float* outDirData = reinterpret_cast<const float*>(outDirImage_->data());
    float* copiedInDirData = reinterpret_cast<float*>(copiedInDirImage_->data());

    float* inDirPtr;
    float* renderedData;
    lb::Vec3f inDir, outDir;
    int numPixels = inDirImage_->s() * inDirImage_->t();
    #pragma omp parallel for private(inDirPtr, renderedData, inDir, outDir)
    for (int i = 0; i < numPixels; ++i) {
        inDirPtr = &inDirData[i * 4];
        renderedData = inDirPtr;

        copiedInDirData[i * 4    ] = inDirPtr[0];
        copiedInDirData[i * 4 + 1] = inDirPtr[1];
        copiedInDirData[i * 4 + 2] = inDirPtr[2];
        copiedInDirData[i * 4 + 3] = inDirPtr[3];

        // Ignore background.
        if (inDirPtr[3] != 0.0f) continue;

        inDir = lb::Vec3f(inDirPtr[0], inDirPtr[1], inDirPtr[2]);
        outDir = lb::Vec3f(outDirData[i * 4], outDirData[i * 4 + 1], outDirData[i * 4 + 2]);

        inDir = inDir.cwiseProduct(lb::Vec3f(2.0, 2.0, 2.0)) - lb::Vec3f(1.0, 1.0, 1.0);
        outDir = outDir.cwiseProduct(lb::Vec3f(2.0, 2.0, 2.0)) - lb::Vec3f(1.0, 1.0, 1.0);
        outDir[2] = std::max(outDir[2], 0.0f);
        outDir.normalize();

        renderedData[0] = renderedData[1] = renderedData[2] = 0.0f;

        if (reflectances_ && environmentIntensity_ > 0.0f) {
            renderReflectance(outDir, renderedData);
        }

        if (dataType_ == lb::BTDF_DATA ||
            dataType_ == lb::SPECULAR_TRANSMITTANCE_DATA) {
            inDir[2] = -inDir[2];
        }

        if (inDir[2] <= 0.0f) continue;
        inDir.normalize();

        if (brdf_ && lightIntensity_ > 0.0f) {
            renderBrdf(inDir, outDir, renderedData);
        }
    }
}

void RenderingDrawCallback::renderBrdf(const lb::Vec3f& inDir,
                                       const lb::Vec3f& outDir,
                                       float*           pixel) const
{
    lb::Spectrum sp =
        brdf_->getSpectrum(inDir.cast<lb::Vec3::Scalar>(), outDir.cast<lb::Vec3::Scalar>());
    const lb::SampleSet* ss = brdf_->getSampleSet();

    lb::Vec3f rgb = lb::SpectrumUtility::toSrgb<lb::Vec3f, lb::SampleSet>(sp, *ss);
    rgb *= lb::PI_F * inDir[2] * lightIntensity_;
    pixel[0] += rgb[0];
    pixel[1] += rgb[1];
    pixel[2] += rgb[2];
}

void RenderingDrawCallback::renderReflectance(const lb::Vec3f& outDir, float* pixel) const
{
    lb::Spectrum sp = reflectances_->getSpectrum(outDir.cast<lb::Vec3::Scalar>());

    lb::Vec3f rgb = lb::SpectrumUtility::toSrgb<lb::Vec3f, lb::SampleSet2D>(sp, *reflectances_);
    rgb *= environmentIntensity_;
    pixel[0] += rgb[0];
    pixel[1] += rgb[1];
    pixel[2] += rgb[2];
}
