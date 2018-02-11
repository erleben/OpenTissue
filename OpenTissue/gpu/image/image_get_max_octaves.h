#ifndef OPENTISSUE_GPU_IMAGE_IMAGE_GET_MAX_OCTAVES_H
#define OPENTISSUE_GPU_IMAGE_IMAGE_GET_MAX_OCTAVES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cmath>

namespace OpenTissue
{
  namespace image
  {

    /**
    * Get Maximum Octaves.
    *
    * @param     image   A reference to the image.
    * @return            The maximum number of octaves that can be seen
    *                    at the current resolution of the image.
    */
    template<typename image_type>
    inline size_t get_max_octaves( image_type const & image)
    {
      using std::max;

      size_t width  = image.width();
      size_t height = image.height();
      size_t pixel_resolution = max(width,height);

      size_t mask = 0;
      if(pixel_resolution<0)
        mask = -pixel_resolution;
      else
        mask = pixel_resolution;

      size_t i=0;
      while(mask>0)
      {
        mask = mask >> 1;
        ++i;
      }
      size_t octaves = max(1u,i-2u);
      return octaves;
    }

  } // namespace image
} // namespace OpenTissue

//OPENTISSUE_GPU_IMAGE_IMAGE_GET_MAX_OCTAVES_H
#endif
