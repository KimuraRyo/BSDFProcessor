// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#ifndef RENDERING_DRAW_CALLBACK_H
#define RENDERING_DRAW_CALLBACK_H

#include <osg/Camera>
#include <osg/Image>

#include <libbsdf/Brdf/Brdf.h>
#include <libbsdf/Brdf/SampleSet2D.h>

/*!
 * \class   RenderingDrawCallback
 * \brief   The RenderingDrawCallback class provides a callback to render an image.
 *
 * Incoming and outgoing directions are stored in images. BRDF, BTDF, reflectance, and transmittance are rendered.
 */
class RenderingDrawCallback : public osg::Camera::DrawCallback
{
public:
    RenderingDrawCallback(osg::Image*       inDirImage,
                          osg::Image*       outDirImage,
                          lb::Brdf*         brdf,
                          lb::SampleSet2D*  reflectances,
                          lb::DataType      dataType,
                          float             lightIntensity,
                          float             environmentIntensity);

    virtual void operator()(const osg::Camera&) const
    {
        render();
    }

private:
    virtual ~RenderingDrawCallback() {}

    /*! Renders an image using inDirImage_ and outDirImage_, and stores the result in inDirImage_. */
    void render() const;

    /*! Renders a pixel with a BRDF and BTDF. */
    void renderBrdf(const lb::Vec3& inDir, const lb::Vec3& outDir, float* pixel) const;
    
    /*! Renders a pixel with a reflectance and transmittance. */
    void renderReflectance(const lb::Vec3& outDir, float* pixel) const;

    osg::Image* inDirImage_;
    osg::Image* outDirImage_;

    lb::Brdf*           brdf_;
    lb::SampleSet2D*    reflectances_;
    lb::DataType        dataType_;

    float lightIntensity_;
    float environmentIntensity_;
};

#endif // RENDERING_DRAW_CALLBACK_H
