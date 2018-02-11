#ifndef OPENTISSUE_GPU_IMAGE_IMAGE_ANIMATE_TURBULENCE_H
#define OPENTISSUE_GPU_IMAGE_IMAGE_ANIMATE_TURBULENCE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/noise/noise_turbulence.h>
#include <cmath>


namespace OpenTissue
{
  namespace image
  {

    /**
    * Animte Turbulence
    *
    * @param image
    * @param time
    * @param freq
    * @param amplitude
    * @param octaves
    */
    template<typename image_type>
    void animate_turbulence( 
        image_type & image
      , float time = 0.0f
      , float freq = 4.0f
      , float amplitude = 0.25f
      , size_t octaves = 4u 
      )
    {
      using std::floor;

      typedef typename image_type::value_type  value_type;

      float z = time - floor(time);
      
      size_t width    = image.width();
      size_t height   = image.height();
      size_t channels = image.channels();
      
      noise::Turbulence<float> f( octaves );

      float x_scale = freq / (width - 1.0);
      float y_scale = freq / (height - 1.0); 

      for(size_t j=0;j<height;++j)
        for(size_t i=0;i<width;++i)
        {
          float x  = x_scale*(i + 0.5);
          float y  = y_scale*(j + 0.5);
          float f0 = f(x, y, z      );
          float f1 = f(x, y, z - 1.0);
          float f  =  f0*(1.0-z) + z*f1;
          value_type value = static_cast<value_type>( f*amplitude );
          for(size_t c=0;c<channels;++c)
            image(i,j,c) = value;
        }
    }

  } // namespace image
} // namespace OpenTissue

//OPENTISSUE_GPU_IMAGE_IMAGE_ANIMATE_TURBULENCE_H
#endif
