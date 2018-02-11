#ifndef OPENTISSUE_GPU_IMAGE_IMAGE_ALPHA_MODULATION_H
#define OPENTISSUE_GPU_IMAGE_IMAGE_ALPHA_MODULATION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/gpu/image/image.h>

namespace OpenTissue
{
  namespace image
  {

    /**
    * Alpha Modulation.
    *
    * @param image      Upon return the alpha channel of this image
    *                   will have been multiplied with the specified
    *                   alpha channel image.
    * @param alpha      A alpha channel image.
    *
    */
    void alpha_modulation( 
      OpenTissue::image::Image<unsigned char> & image
      , OpenTissue::image::Image<unsigned char> const & alpha 
      )
    {
      using std::min;

      assert(image.channels()==4               || !"alpha_modulation(): image must have four channels");
      assert(alpha.channels()==1               || !"alpha_modulation(): alpha must have 1 channels");
      assert( image.width() == alpha.width()   || !"alpha_modulation(): alpha and image dimensions did not fit");
      assert( image.height() == alpha.height() || !"alpha_modulation(): alpha and image dimensions did not fit");

      unsigned char * ival = static_cast<unsigned char*>( image.get_data() );
      unsigned char const * aval = static_cast<unsigned char const *>( alpha.get_data() );

      size_t pixels = image.width()*image.height();

      float factor = 1.0f/255.0f;

      for(size_t i=0;i<pixels;++i)
      {
        float a = factor*(*aval++);
        float ir = factor*(*(ival));
        float ig = factor*(*(ival+1));
        float ib = factor*(*(ival+2));
        float ia = factor*(*(ival+3));

        *ival++ = static_cast<unsigned char>( min(ir,1.0f)*255.0f );
        *ival++ = static_cast<unsigned char>( min(ig,1.0f)*255.0f );
        *ival++ = static_cast<unsigned char>( min(ib,1.0f)*255.0f );
        *ival++ = static_cast<unsigned char>( min(ia*a,1.0f)*255.0f );
      }
    }

  } // namespace image
} // namespace OpenTissue

//OPENTISSUE_GPU_IMAGE_IMAGE_ALPHA_MODULATION_H
#endif
