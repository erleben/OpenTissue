#ifndef OPENTISSUE_COLLISION_SDF_SDF_DEBUG_DRAW_SAMPLING_H
#define OPENTISSUE_COLLISION_SDF_SDF_DEBUG_DRAW_SAMPLING_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/graphics/core/gl/gl_util.h>

namespace OpenTissue
{
  namespace sdf
  {

    /**
    * Sampling Debug Drawing Utility.
    * This function can be used to draw point sampling  of a signed distance
    * field geometry.
    *
    * @param geometry     The signed distance field geometry.
    */
    template<typename sdf_geometry_type>
    void debug_draw_sampling(sdf_geometry_type const & geometry) 
    {
      typedef typename sdf_geometry_type::const_point_iterator    const_point_iterator;
      const_point_iterator end = geometry.m_sampling.end();
      const_point_iterator s   = geometry.m_sampling.begin();
      for(;s != end ;++s)
        gl::DrawPoint( (*s) );
    }

  } // namespace sdf

} // namespace OpenTissue

// OPENTISSUE_COLLISION_SDF_SDF_DEBUG_DRAW_SAMPLING_H
#endif
