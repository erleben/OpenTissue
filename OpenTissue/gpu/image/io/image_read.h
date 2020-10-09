//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <png.h>

#include <iostream>
#include <string>
#include <vector>

#include <OpenTissue/configuration.h>
#include <OpenTissue/gpu/image/image.h>

namespace OpenTissue {
namespace image {
/**
* Read Image.
*
* @param   filename
* @param   image
* @return               If succesful then the return value is true
*                       otherwise it is false.
*/
bool read(std::string const & filename,
          OpenTissue::image::Image<unsigned char> & image,
          bool show_statistics = true)
{
  // If you have enabled a texture before invoking this function
  // then ilu fucks up textures, so we need to make sure that the
  // openGL default texture is enabled and not one of yours...
  // glBindTexture( GL_TEXTURE_2D, 0 );

  auto fp = fopen(filename.c_str(), "rb");
  if(!fp)
  {
      std::cerr << "error: failed to open png file " << filename << "." << std::endl;
      return false;
  }

  // Read the header
  png_byte header[8];
  fread(header, 1, 8, fp);

  if (png_sig_cmp(header, 0, 8))
  {
      std::cerr << "error: " << filename << " is not a PNG." << std::endl;
      fclose(fp);
      return false;
  }

  png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if(!png)
  {
    std::cerr << "error: png_create_read_struct returned 0." << std::endl;
    fclose(fp);
    return false;
  }

  png_infop info = png_create_info_struct(png);
  if(!info)
  {
    std::cerr << "error: png_create_info_struct returned 0." << std::endl;
    png_destroy_read_struct(&png, nullptr, nullptr);
    fclose(fp);
    return false;
  }

  // Create png end info struct
  png_infop end_info = png_create_info_struct(png);
  if (!end_info)
  {
      std::cerr << "error: png_create_info_struct returned 0" << std::endl;
      png_destroy_read_struct(&png, &info, nullptr);
      fclose(fp);
      return false;
  }


  if (setjmp(png_jmpbuf(png))) {
      std::cerr << "error from libpng" << std::endl;
      png_destroy_read_struct(&png, &info, &end_info);
      fclose(fp);
      return false;
  }

  // Bind io function for the PNG file to the stream
  png_init_io(png, fp);

  // Let the read functions know that we have already read the 1st 8 bytes
  png_set_sig_bytes(png, 8);

  // Read metadata
  png_read_info(png, info);

  // Fetch metadata
  png_uint_32 width;
  png_uint_32 height;
  int color_type;
  int bit_depth;
  int interlaced_method;
  int compression_method;
  int filter_method;
  png_get_IHDR(png, info, &width, &height, &bit_depth, &color_type, &interlaced_method, &compression_method, &filter_method);

  auto channels   = png_get_channels(png, info);

  if (show_statistics)
  {
    std::cout << "--- file : " << filename << "---------------------------" << std::endl;
    std::cout << "\twidth               = " << width << std::endl;
    std::cout << "\theight              = " << height << std::endl;
    std::cout << "\tdepth               = " << bit_depth << std::endl;
    std::cout << "\tinterlaced_method   = " << interlaced_method << std::endl;
    std::cout << "\tcompression_method  = " << compression_method << std::endl;
    std::cout << "\tfilter_method       = " << filter_method << std::endl;
  }

  if(bit_depth == 16)
  {
    png_set_strip_16(png);
  }

  if(color_type == PNG_COLOR_TYPE_PALETTE)
  {
    png_set_palette_to_rgb(png);
  }

  // PNG_COLOR_TYPE_GRAY_ALPHA is always 8 or 16bit depth.
  if(color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
  {
    png_set_expand_gray_1_2_4_to_8(png);
  }

  if(png_get_valid(png, info, PNG_INFO_tRNS))
  {
    png_set_tRNS_to_alpha(png);
  }

  // If color_type doesn't have an alpha channel then fill it with 0xff.
  if(color_type == PNG_COLOR_TYPE_RGB || color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_PALETTE)
  {
    png_set_filler(png, 0xFF, PNG_FILLER_AFTER);
  }

  if(color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
  {
    png_set_gray_to_rgb(png);
  }

  png_read_update_info(png, info);

  image.create(width, height, channels);
  auto data = static_cast<unsigned char*>(image.get_data());

  // Set row pointers to the correct offset of image data
  size_t rowbytes = png_get_rowbytes(png, info);
  std::vector<png_bytep> row_pointers(height);
  for (png_uint_32 row = 0; row < height; ++row)
  {
    row_pointers[row] = data + row * rowbytes;
  }

  // Read the png into image_data through row_pointers
  png_read_image(png, row_pointers.data());

  png_destroy_read_struct(&png, &info, &end_info);
  fclose(fp);

  return true;
}

} // namespace image
} // namespace OpenTissue
