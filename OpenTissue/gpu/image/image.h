#ifndef OPENTISSUE_GPU_IMAGE_IMAGE_H
#define OPENTISSUE_GPU_IMAGE_IMAGE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/gpu/texture/texture_texture2D.h>
#include <OpenTissue/gpu/texture/texture_types.h>

#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>

namespace OpenTissue
{
  namespace image
  {

    /**
    * Image Class.
    */
    template<typename T>
    class Image
    {
    public:

      typedef T  value_type;

    protected:

      size_t          m_width;
      size_t          m_height;
      size_t          m_channels;
      std::vector<T>  m_data;

    public:

      size_t width()    const { return m_width;    }
      size_t height()   const { return m_height;   }
      size_t channels() const { return m_channels; }

      T & operator()(size_t x,size_t y,size_t channel)
      {
        return m_data[(y*m_width + x)*m_channels + channel];
      }

      T const & operator()(size_t x,size_t y,size_t channel) const
      {
        return m_data[(y*m_width + x)*m_channels + channel];
      }

      void       * get_data()       { return &(m_data[0]); }
      void const * get_data() const { return &(m_data[0]); }

    public:

      Image()
        :  m_width(0)
        ,  m_height(0)
        ,  m_channels(0)
        ,  m_data(0)
      {}

      Image( size_t width, size_t height, size_t channels, T const * data)
        : m_width(width)
        , m_height(height)
        , m_channels(channels)
        , m_data( data, data+(width*height*channels) )
      {}

      Image( size_t width, size_t height, size_t channels,  std::vector<T> const & data )
        : m_width(width)
        , m_height(height)
        , m_channels(channels)
        , m_data(data)
      {}

      Image( size_t width, size_t height, size_t channels )
        : m_width(width)
        , m_height(height)
        , m_channels(channels)
        , m_data(width*height*channels)
      {}

    public:

      void create( size_t width, size_t height, size_t channels )
      {
        m_width    = width;
        m_height   = height;
        m_channels = channels;
        m_data.resize(width*height*channels);
        std::fill(m_data.begin(),m_data.end(),static_cast<T>(0));
      }

      /**
      *
      * @param rectangular     Set to true if a rectangular texture is wanted,
      *                        default is false.
      * @param border          Set border value, default is zero
      */
      OpenTissue::texture::texture2D_pointer create_texture(  
          size_t internal_format
        , bool rectangular=false
        , int border=0
        ) const
      {
        size_t ext_format = OpenTissue::texture::external_format( m_channels );
        OpenTissue::texture::texture2D_pointer tex(
          new OpenTissue::texture::Texture2D(
          internal_format
          , m_width
          , m_height
          , ext_format
          , OpenTissue::texture::external_type<T>()
          , &m_data[0]
          , rectangular
          , border
          )
          );
        return tex;
      }

    };

  } // namespace image
} // namespace OpenTissue

//OPENTISSUE_GPU_IMAGE_IMAGE_H
#endif
