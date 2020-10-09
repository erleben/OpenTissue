//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

#include <png.h>
#include <vector>
#include <iostream>
#include <string>

#include <OpenTissue/configuration.h>
#include <OpenTissue/gpu/image/image.h>

namespace OpenTissue {
namespace image {
/**
* Write Image.
*
* @param   filename
* @param   image
* @return  If successful then the return value is true
*          otherwise it is false.
*/
bool write(std::string const & filename,
           OpenTissue::image::Image<unsigned char> const & image)
{
  png_uint_32 width = image.width();
  png_uint_32 height = image.height();
  size_t channels = image.channels();

  if(channels > 4)
  {
    std::cerr << "error: can't write PNG files that have other than 1, 2, 3, or 4 channels." << std::endl;
    return false;
  }

  // open file
  auto fp = fopen(filename.c_str(), "wb");
  if(!fp)
  {
    std::cerr << "error: file " << filename << " could not be opened for writing." << std::endl;
    return false;
  }

  png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if(!png)
  {
    std::cerr << "error: png_create_write_struct returned 0." << std::endl;
    fclose(fp);
    return false;
  }

  // allocate/initialize the image information data.
  png_infop info = png_create_info_struct(png);
  if (info == NULL)
  {
    fclose(fp);
    png_destroy_write_struct(&png, nullptr);
    return false;
  }

  // set error handling.  REQUIRED if you aren't supplying your own
  // error handling functions in the png_create_write_struct() call.
  if (setjmp(png_jmpbuf(png)))
  {
    /* If we get here, we had a problem reading the file */
    fclose(fp);
    png_destroy_write_struct(&png, &info);
    return false;
  }

  // set up the output control if you are using standard C streams
  png_init_io(png, fp);

  unsigned int bit_depth = 16;
  if (sizeof(uint8_t) == 1)
  {
    bit_depth = 8;
  }

  // Set the image information here.  Width and height are up to 2^31,
  // bit_depth is one of 1, 2, 4, 8, or 16, but valid values also depend on
  // the color_type selected. color_type is one of PNG_COLOR_TYPE_GRAY,
  // PNG_COLOR_TYPE_GRAY_ALPHA, PNG_COLOR_TYPE_PALETTE, PNG_COLOR_TYPE_RGB,
  // or PNG_COLOR_TYPE_RGB_ALPHA.  interlace is either PNG_INTERLACE_NONE or
  // PNG_INTERLACE_ADAM7, and the compression_type and filter_type MUST
  // currently be PNG_COMPRESSION_TYPE_BASE and PNG_FILTER_TYPE_BASE. REQUIRED
  png_set_IHDR(png, info, width, height, bit_depth,
               PNG_COLOR_TYPE_RGB_ALPHA,
               PNG_INTERLACE_NONE,
               PNG_COMPRESSION_TYPE_BASE,
               PNG_FILTER_TYPE_BASE);

  png_write_info(png, info);

  png_bytep pixel = static_cast<png_bytep>(const_cast<void*>(image.get_data()));
  std::vector<png_bytep> row_pointers(height);

  for (png_uint_32 row = 0; row < height; ++row)
  {
    row_pointers[row] = pixel + row * width * channels;
  }

  png_write_image(png, row_pointers.data());
  png_write_end(png, info);
  png_destroy_write_struct(&png, &info);
  fclose(fp);

  return true;
}

} // namespace image
} // namespace OpenTissue
