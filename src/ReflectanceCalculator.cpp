// =================================================================== //
// Copyright (C) 2016-2019 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "ReflectanceCalculator.h"

#include <iostream>

#include <QApplication>
#include <QThread>

#include <osg/Timer>

#include <libbsdf/Brdf/Analyzer.h>
#include <libbsdf/Brdf/Integrator.h>
#include <libbsdf/Brdf/SphericalCoordinatesBrdf.h>
#include <libbsdf/Brdf/SpecularCoordinatesBrdf.h>

#include <libbsdf/Common/PoissonDiskDistributionOnSphere.h>

ReflectanceCalculator::ReflectanceCalculator(lb::SampleSet2D*                   reflectances,
                                             const std::shared_ptr<lb::Brdf>    brdf)
                                             : reflectances_(reflectances),
                                               brdf_(brdf),
                                               stopped_(false)
{
    intialize(reflectances);
}

ReflectanceCalculator::ReflectanceCalculator(lb::SampleSet2D*                   reflectances,
                                             const std::shared_ptr<lb::Btdf>    btdf)
                                             : reflectances_(reflectances),
                                               btdf_(btdf),
                                               stopped_(false)
{
    intialize(reflectances);
}

ReflectanceCalculator::~ReflectanceCalculator()
{
    delete processedReflectances_;
}

void ReflectanceCalculator::stop()
{
    stopped_ = true;
}

void ReflectanceCalculator::computeReflectances()
{
    std::cout
        << "[ReflectanceCalculator::computeReflectances] thread: "
        << QThread::currentThread() << std::endl;

    lb::Brdf* brdf;
    if (brdf_) {
        brdf = brdf_.get();
    }
    else if (btdf_) {
        brdf = btdf_->getBrdf();
    }
    else {
        return;
    }

    osg::Timer_t startTick = osg::Timer::instance()->tick();

    lb::Integrator integrator = lb::Integrator(lb::PoissonDiskDistributionOnSphere::NUM_SAMPLES_ON_HEMISPHERE, true);
    //lb::Integrator integrator = lb::Integrator(10000000, false);

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

        if (lb::SphericalCoordinatesBrdf* spheBrdf = dynamic_cast<lb::SphericalCoordinatesBrdf*>(brdf)) {
            sp = lb::computeReflectance(*spheBrdf, inThIndex, inPhIndex);
        }
        else if (lb::SpecularCoordinatesBrdf* specBrdf = dynamic_cast<lb::SpecularCoordinatesBrdf*>(brdf)) {
            sp = lb::computeReflectance(*specBrdf, inThIndex, inPhIndex);
        }
        else {
            inDir = processedReflectances_->getDirection(inThIndex, inPhIndex);
            sp = integrator.computeReflectance(*brdf, inDir);
        }

        processedReflectances_->setSpectrum(inThIndex, inPhIndex, sp);

        qApp->processEvents();
    }}

    osg::Timer_t endTick = osg::Timer::instance()->tick();
    double delta = osg::Timer::instance()->delta_s(startTick, endTick);
    std::cout << "[ReflectanceCalculator::computeReflectances] " << delta << "(s)" << std::endl;

    reflectances_->getSpectra() = processedReflectances_->getSpectra();

    emit finished();
}

void ReflectanceCalculator::intialize(lb::SampleSet2D* reflectances)
{
    processedReflectances_ = new lb::SampleSet2D(reflectances->getNumTheta(),
                                                 reflectances->getNumPhi(),
                                                 reflectances->getColorModel(),
                                                 reflectances->getNumWavelengths());

    processedReflectances_->getWavelengths() = reflectances->getWavelengths();
    processedReflectances_->getThetaArray()  = reflectances->getThetaArray();
    processedReflectances_->getPhiArray()    = reflectances->getPhiArray();
}
