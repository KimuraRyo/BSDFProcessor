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

    osg::Image* getInDirImage() { return copiedInDirImage_.get(); }
    osg::Image* getOutDirImage() { return outDirImage_; }

private:
    virtual ~RenderingDrawCallback() {}

    /*! Renders an image using inDirImage_ and outDirImage_, and stores the result in inDirImage_. */
    void render() const;

    /*! Renders a pixel with a BRDF and BTDF. */
    void renderBrdf(const lb::Vec3& inDir, const lb::Vec3& outDir, float* pixel) const;
    
    /*! Renders a pixel with a reflectance and transmittance. */
    void renderReflectance(const lb::Vec3& outDir, float* pixel) const;

    osg::Image* inDirImage_;    /*!< Incoming directions as input and the renderd image as output. */
    osg::Image* outDirImage_;   /*!< Outgoing directions. */

    lb::Brdf*           brdf_;
    lb::SampleSet2D*    reflectances_;
    lb::DataType        dataType_;

    float lightIntensity_;
    float environmentIntensity_;

    /*!< Incoming directions copied from \a inDirImage_. This property is used for picking. */
    osg::ref_ptr<osg::Image> copiedInDirImage_;
};

#endif // RENDERING_DRAW_CALLBACK_H
