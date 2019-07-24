// =================================================================== //
// Copyright (C) 2016-2019 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef REFLECTANCE_CALCULATOR_H
#define REFLECTANCE_CALCULATOR_H

#include <memory>

#include <QObject>

#include <libbsdf/Brdf/Brdf.h>
#include <libbsdf/Brdf/Btdf.h>
#include <libbsdf/Brdf/SampleSet2D.h>

/*!
 * \class   ReflectanceCalculator
 * \brief   The ReflectanceCalculator class provides a worker object to compute reflectances.
 */
class ReflectanceCalculator : public QObject
{
    Q_OBJECT

public:
    ReflectanceCalculator(lb::SampleSet2D*                  reflectances,
                          const std::shared_ptr<lb::Brdf>   brdf);

    ReflectanceCalculator(lb::SampleSet2D*                  reflectances,
                          const std::shared_ptr<lb::Btdf>   btdf);

    ~ReflectanceCalculator();

public slots:
    void computeReflectances();
    void stop();

signals:
    void finished();
    void stopped();

private:
    void intialize(lb::SampleSet2D* reflectances);

    std::shared_ptr<lb::Brdf> brdf_;
    std::shared_ptr<lb::Btdf> btdf_;

    lb::SampleSet2D* reflectances_; /*!< Reflectances at each incoming direction. */

    /*! Reflectances at each incoming direction processed in another thread. */
    lb::SampleSet2D* processedReflectances_;

    bool stopped_;
};

#endif // REFLECTANCE_CALCULATOR_H
