// =================================================================== //
// Copyright (C) 2016 Kimura Ryo                                       //
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

ReflectanceCalculator::ReflectanceCalculator(lb::SampleSet2D*                   reflectances,
                                             const std::shared_ptr<lb::Brdf>    brdf,
                                             lb::Integrator*                    integrator)
                                             : reflectances_(reflectances),
                                               brdf_(brdf),
                                               integrator_(integrator),
                                               stopped_(false)
{
    intialize(reflectances, integrator);
}

ReflectanceCalculator::ReflectanceCalculator(lb::SampleSet2D*                   reflectances,
                                             const std::shared_ptr<lb::Btdf>    btdf,
                                             lb::Integrator*                    integrator)
                                             : reflectances_(reflectances),
                                               btdf_(btdf),
                                               integrator_(integrator),
                                               stopped_(false)
{
    intialize(reflectances, integrator);
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

    // Compute reflectance or transmittance.
    for (int inPhIndex = 0; inPhIndex < processedReflectances_->getNumPhi();   ++inPhIndex) {
    for (int inThIndex = 0; inThIndex < processedReflectances_->getNumTheta(); ++inThIndex) {
        if (stopped_) {
            emit stopped();
            return;
        }

        lb::Vec3 inDir = processedReflectances_->getDirection(inThIndex, inPhIndex);
        lb::Spectrum sp = integrator_->computeReflectance(*brdf, inDir);
        processedReflectances_->setSpectrum(inThIndex, inPhIndex, sp);

        qApp->processEvents();
    }}

    osg::Timer_t endTick = osg::Timer::instance()->tick();
    double delta = osg::Timer::instance()->delta_s(startTick, endTick);
    std::cout << "[ReflectanceCalculator::computeReflectances] " << delta << "(s)" << std::endl;

    reflectances_->getSpectra() = processedReflectances_->getSpectra();

    emit finished();
}

void ReflectanceCalculator::intialize(lb::SampleSet2D* reflectances, lb::Integrator* integrator)
{
    processedReflectances_ = new lb::SampleSet2D(reflectances->getNumTheta(),
                                                 reflectances->getNumPhi(),
                                                 reflectances->getColorModel(),
                                                 reflectances->getNumWavelengths());

    processedReflectances_->getWavelengths() = reflectances->getWavelengths();
    processedReflectances_->getThetaArray() = reflectances->getThetaArray();
    processedReflectances_->getPhiArray() = reflectances->getPhiArray();
}
