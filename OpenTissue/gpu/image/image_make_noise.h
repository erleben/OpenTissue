#ifndef OPENTISSUE_GPU_IMAGE_IMAGE_MAKE_NOISE_H
#define OPENTISSUE_GPU_IMAGE_IMAGE_MAKE_NOISE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/noise/noise_improved_perlin.h>


namespace OpenTissue
{
  namespace image
  {

    /**
     * Fill Image with Noise.
     *
     * @param image      Upon return this image will contain the
     *                   generated noise-image.
     * @param freq       The frequency of the noise.
     * @param amplitude  The amplitude of the noise.
     */
    template<typename image_type>
    void make_noise( 
        image_type & image
      , float freq=4.0f
      , float amplitude = 1.0f 
      )
    {
      typedef typename image_type::value_type  value_type;

      size_t width    = image.width();
      size_t height   = image.height();
      size_t channels = image.channels();
      
      noise::ImprovedPerlinNoise<float> f;
      
      float x_scale = freq / (width-1.0);
      float y_scale = freq / (height-1.0); 
      
      for(size_t j=0;j<height;++j)
        for(size_t i=0;i<width;++i)
        {
          float x  = x_scale*(i + 0.5);
          float y  = y_scale*(j + 0.5);
          float biased = (f(x,y)+1.0)*.5;
          value_type value = static_cast<value_type>( biased*amplitude );
          for(size_t c=0;c<channels;++c)
            image(i,j,c) = value;
        }
    }

  } // namespace image
} // namespace OpenTissue

//OPENTISSUE_GPU_IMAGE_IMAGE_MAKE_NOISE_H
#endif
