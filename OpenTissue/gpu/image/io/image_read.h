#ifndef OPENTISSUE_GPU_IMAGE_IO_IMAGE_READ_H
#define OPENTISSUE_GPU_IMAGE_IO_IMAGE_READ_H
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
    * Read Image.
    *
    * @param   filename
    * @param   image
    * @return               If succesful then the return value is true
    *                       otherwise it is false.
    */
    bool read(
      std::string const & filename
      , OpenTissue::image::Image<unsigned char> & image
      , bool show_statistics = true
      )
    {
      // If you have enabled a texture before invoking this function
      // then ilu fucks up textures, so we need to make sure that the
      // openGL default texture is enabled and not one of yours...    
      glBindTexture( GL_TEXTURE_2D, 0 );

      ilInit();
      iluInit();
      ilutInit();

      //ilEnable (IL_ORIGIN_SET);
      //ilSetInteger (IL_ORIGIN_MODE, IL_ORIGIN_LOWER_LEFT);
      
      OpenTissue::image::detail::ilImage devil;

      ILboolean result = devil.Load(const_cast< char * >(filename.c_str()));
      if (!result)
      {
        return false;
      }

      if (show_statistics)
      {
        std::cout << "--- file : " << filename << "---------------------------" << std::endl;
        std::cout << "\twidth         = " << (int)devil.Width() << std::endl;
        std::cout << "\theight        = " << (int)devil.Height() << std::endl;
        std::cout << "\tdepth         = " << (int)devil.Depth() << std::endl;
        std::cout << "\tBpp           = " << (int)devil.Bpp() << std::endl;
        std::cout << "\tBitpp         = " << (int)devil.Bitpp() << std::endl;
        std::cout << "\tPallette Type = " << OpenTissue::image::detail::get_IL_string(devil.PaletteType()) << std::endl;
        std::cout << "\tFormat        = " << OpenTissue::image::detail::get_IL_string(devil.Format()) << std::endl;
        std::cout << "\tType          = " << OpenTissue::image::detail::get_IL_string(devil.Type()) << std::endl;
        std::cout << "\tNumImages     = " << (int)devil.NumImages() << std::endl;
        std::cout << "\tNumMipmaps    = " << (int)devil.NumMipmaps() << std::endl;
        std::cout << "\tOrigin        = " << OpenTissue::image::detail::get_IL_string(devil.GetOrigin()) << std::endl;
      }

      if (devil.GetOrigin() != IL_ORIGIN_LOWER_LEFT)
        devil.Flip();

      size_t width  = devil.Width();
      size_t height = devil.Height();
      image.create(width, height, 4);

      devil.Bind();
      ilCopyPixels(0, 0, 0, width, height, 1, IL_RGBA, IL_UNSIGNED_BYTE, image.get_data() );

      ilShutDown();

      return true;
    }

  } // namespace image
} // namespace OpenTissue

//OPENTISSUE_GPU_IMAGE_IO_IMAGE_READ_H
#endif
