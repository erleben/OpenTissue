#ifndef OPENTISSUE_GPU_IMAGE_IMAGE_MAKE_FRACTAL_SUM_NOISE_H
#define OPENTISSUE_GPU_IMAGE_IMAGE_MAKE_FRACTAL_SUM_NOISE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/noise/noise_fractal_sum.h>


namespace OpenTissue
{
  namespace image
  {

    /**
    * Genereate Fractal Sum Noise Image.
    *
    * @param image            A refernece to the image that upon return will
    *                         be filled with the fractal sum noise.
    * @param freq             The frequency of the noise.
    * @param amplitude        The amplitude of the noise.
    * @param octaves          The numer of octaves to use.
    */
    template<typename image_type>
    void make_fractal_sum_noise( 
      image_type & image
      , float freq=4.0f
      , float amplitude = 0.25f
      , size_t octaves = 4u
      )
    {
      typedef typename image_type::value_type  value_type;

      size_t width    = image.width();
      size_t height   = image.height();
      size_t channels = image.channels();

      noise::FractalSumNoise<float> f( octaves );

      float x_scale = freq / (width-1.0);
      float y_scale = freq / (height-1.0); 

      for(size_t j=0;j<height;++j)
        for(size_t i=0;i<width;++i)
        {
          float x  = x_scale*(i + 0.5f);
          float y  = y_scale*(j + 0.5f);
          float biased = (f(x,y) + 1.0f)*.5f;

          value_type value = static_cast<value_type>( biased*amplitude );

          for(size_t c=0;c<channels;++c)
            image(i,j,c) = value;
        }
    }

  } // namespace image
} // namespace OpenTissue

//OPENTISSUE_GPU_IMAGE_IMAGE_MAKE_FRACTAL_SUM_NOISE_H
#endif
