#ifndef OPENTISSUE_GPU_IMAGE_IMAGE_SCREEN_CAPTURE_H
#define OPENTISSUE_GPU_IMAGE_IMAGE_SCREEN_CAPTURE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/gl/gl_util.h>
#include <OpenTissue/gpu/image/image.h>

#include <boost/shared_ptr.hpp>

namespace OpenTissue
{
  namespace image
  {

    /**
    *  Tags used for type dispatching of screen_capture
    */
    struct keep_transparency {};
    struct no_transparency {};

    /**
    * Screen Capture.
    * This function keeps transparency information from the frame buffer.
    *
    * @return    A pointer to an image containing the captured screen.
    */
    boost::shared_ptr<OpenTissue::image::Image<unsigned char> >  screen_capture( keep_transparency const & /*tag*/ )
    {
      typedef boost::shared_ptr<OpenTissue::image::Image<unsigned char> > image_pointer;

      GLint viewport[4];
      glGetIntegerv(GL_VIEWPORT,viewport);

      bool origin_top_left = false; // glReadPixels image origin is bottom-left
      image_pointer image( new OpenTissue::image::Image<unsigned char>( viewport[2], viewport[3], 4, origin_top_left ) );
      glReadPixels(viewport[0], viewport[1], viewport[2], viewport[3], GL_RGBA, GL_UNSIGNED_BYTE, image->get_data() );
      return image;
    }

    /**
    * Screen Capture.
    * Ignores any transparency information stored in the frame buffer.
    *
    * This function was contributed by Christian Iversen, due to
    * problems with ATI drivers on linux/windows.
    *
    * @return    A pointer to an image containing the captured screen.
    */
    boost::shared_ptr<OpenTissue::image::Image<unsigned char> >  screen_capture( no_transparency const & /*tag*/ )
    {
      typedef boost::shared_ptr<OpenTissue::image::Image<unsigned char> > image_pointer;

      GLint viewport[4];
      glGetIntegerv(GL_VIEWPORT,viewport);

      // This is width * height
      int count = viewport[2]*viewport[3];

      std::vector<unsigned char> pixels4(4*count);
      std::vector<unsigned char> pixels3(3*count);

      // Load the pixel data into the RGB struct array
      glReadPixels(viewport[0], viewport[1], viewport[2], viewport[3], GL_RGB, GL_UNSIGNED_BYTE, (&pixels3[0]));

      // Convert the RGB data to RGBA data, setting alpha to the maximum value
      for (int x = 0; x < count; x++)
      {
        pixels4[x*4]   = pixels3[x*3];
        pixels4[x*4+1] = pixels3[x*3+1];
        pixels4[x*4+2] = pixels3[x*3+2];
        pixels4[x*4+3] = 255;
      }

      // Write the RGBA data
      unsigned char * raw_ptr = static_cast< unsigned char * > (&pixels4[0]);

      bool origin_top_left = false; // glReadPixels image origin is bottom-left
      image_pointer image(new OpenTissue::image::Image<unsigned char>(viewport[2], viewport[3], 4, raw_ptr, origin_top_left));
      return image;
    }

    /**
    * Screen Capture.
    * Uses the keep_transparency mode by default.
    *
    * @return    A pointer to an image containing the captured screen.
    */
    boost::shared_ptr<OpenTissue::image::Image<unsigned char> >  screen_capture(  )
    {
      return screen_capture( keep_transparency() );
    }

  } // namespace image
} // namespace OpenTissue

//OPENTISSUE_GPU_IMAGE_IMAGE_SCREEN_CAPTURE_H
#endif
