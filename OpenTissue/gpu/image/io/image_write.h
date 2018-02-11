#ifndef OPENTISSUE_GPU_IMAGE_IO_IMAGE_WRITE_H
#define OPENTISSUE_GPU_IMAGE_IO_IMAGE_WRITE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/gpu/image/image.h>
#include <OpenTissue/gpu/image/io/image_il_wrap.h>

#include <iostream>
#include <string>

namespace OpenTissue
{
  namespace image
  {
    /**
    * Write Image.
    *
    * @param   filename
    * @param   image
    * @return               If succesful then the return value is true
    *                       otherwise it is false.
    */
    bool write(
        std::string const & filename
      , OpenTissue::image::Image<unsigned char> const & image
      )
    {
      ilInit();
      iluInit();
      ilutInit();
      
      //ilEnable (IL_ORIGIN_SET);
      //ilSetInteger (IL_ORIGIN_MODE, IL_ORIGIN_LOWER_LEFT);
      
      OpenTissue::image::detail::ilImage devil;

      size_t width = image.width();
      size_t height = image.height();
      
      unsigned char * pixel = static_cast<unsigned char *>( const_cast<void *>(image.get_data()) );

      devil.TexImage(width, height, 1, 4, IL_RGBA, IL_UNSIGNED_BYTE, pixel);

      ilEnable(IL_FILE_OVERWRITE);

      bool result = !(0 == devil.Save(const_cast< char * >(filename.c_str())) );

      ilShutDown();

      return result;
    }

  } // namespace image
} // namespace OpenTissue

//OPENTISSUE_GPU_IMAGE_IO_IMAGE_WRITE_H
#endif
