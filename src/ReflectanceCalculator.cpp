// =================================================================== //
// Copyright (C) 2016-2020 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "ReflectanceCalculator.h"

#include <QApplication>
#include <QThread>

#include <osg/Timer>

#include <libbsdf/Brdf/Analyzer.h>

#if defined(USE_INTEGRATOR)
#include <libbsdf/Brdf/Integrator.h>
#endif

ReflectanceCalculator::ReflectanceCalculator(std::shared_ptr<lb::SampleSet2D>   reflectances,
                                             std::shared_ptr<const lb::Brdf>    brdf)
                                             : reflectances_(reflectances),
                                               brdf_(brdf),
                                               stopped_(false)
{
    intialize(reflectances.get());
}

ReflectanceCalculator::ReflectanceCalculator(std::shared_ptr<lb::SampleSet2D>   reflectances,
                                             std::shared_ptr<const lb::Btdf>    btdf)
                                             : reflectances_(reflectances),
                                               btdf_(btdf),
                                               stopped_(false)
{
    intialize(reflectances.get());
}

void ReflectanceCalculator::stop()
{
    stopped_ = true;
}

void ReflectanceCalculator::computeReflectances()
{
    lbInfo << "[ReflectanceCalculator::computeReflectances] Thread: " << QThread::currentThread();

    const lb::Brdf* brdf;
    if (brdf_) {
        brdf = brdf_.get();
    }
    else if (btdf_) {
        brdf = btdf_->getBrdf().get();
    }
    else {
        return;
    }

    osg::Timer_t startTick = osg::Timer::instance()->tick();

#if defined(USE_INTEGRATOR)
    lb::Integrator integrator = lb::Integrator(10000000);
#endif

    // Compute reflectances or transmittances.
    lb::Spectrum sp;
    lb::Vec3 inDir;
    int inPhIndex;
    #pragma omp parallel for private(sp, inDir, inPhIndex) schedule(dynamic)
    for (int inThIndex = 0; inThIndex < processedReflectances_->getNumTheta(); ++inThIndex) {
    for (    inPhIndex = 0; inPhIndex < processedReflectances_->getNumPhi();   ++inPhIndex) {
        if (stopped_) {
            emit stopped();
            continue;
        }

#if defined(USE_INTEGRATOR)
        sp = integrator.computeReflectance(*brdf, inDir);
#else
        auto spheBrdf = dynamic_cast<const lb::SphericalCoordinatesBrdf*>(brdf);
        auto specBrdf = dynamic_cast<const lb::SpecularCoordinatesBrdf*>(brdf);
        if (spheBrdf &&
            spheBrdf->getNumOutTheta() >= 2 &&
            spheBrdf->getNumOutPhi() >= 2) {
            sp = lb::computeReflectance(*spheBrdf, inThIndex, inPhIndex);
        }
        else if (specBrdf &&
                 specBrdf->getNumSpecTheta() >= 2 &&
                 specBrdf->getNumSpecPhi() >= 2) {
            sp = lb::computeReflectance(*specBrdf, inThIndex, inPhIndex);
        }
        else {
            inDir = processedReflectances_->getDirection(inThIndex, inPhIndex);

            static lb::Log::Level origLogLevel = lb::Log::getNotificationLevel();
            lb::Log::setNotificationLevel(lb::Log::Level::WARN_MSG);

            sp = lb::computeReflectance(*brdf, inDir);

            lb::Log::setNotificationLevel(origLogLevel);
        }
#endif

        processedReflectances_->setSpectrum(inThIndex, inPhIndex, sp);

        qApp->processEvents();
    }}

    osg::Timer_t endTick = osg::Timer::instance()->tick();
    double delta = osg::Timer::instance()->delta_s(startTick, endTick);
    lbInfo << "[ReflectanceCalculator::computeReflectances] " << delta << "(s)";

    reflectances_->getSpectra() = processedReflectances_->getSpectra();

    emit finished();
}

void ReflectanceCalculator::intialize(lb::SampleSet2D* reflectances)
{
    processedReflectances_.reset(new lb::SampleSet2D(reflectances->getNumTheta(),
                                                     reflectances->getNumPhi(),
                                                     reflectances->getColorModel(),
                                                     reflectances->getNumWavelengths()));

    processedReflectances_->getWavelengths() = reflectances->getWavelengths();
    processedReflectances_->getThetaArray()  = reflectances->getThetaArray();
    processedReflectances_->getPhiArray()    = reflectances->getPhiArray();
}
