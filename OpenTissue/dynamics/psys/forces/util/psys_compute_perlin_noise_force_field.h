#ifndef OPENTISSUE_DYNAMICS_PSYS_FORCE_UTIL_PSYS_COMPUTE_PERLIN_NOISE_FORCE_FIELD_H
#define OPENTISSUE_DYNAMICS_PSYS_FORCE_UTIL_PSYS_COMPUTE_PERLIN_NOISE_FORCE_FIELD_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/grid/util/grid_idx2coord.h>
#include <OpenTissue/core/math/noise/noise_perlin.h>


namespace OpenTissue
{
  namespace psys
  {

    template<typename grid_type>
      void compute_perlin_noise_force_field( grid_type & field )
    {      
      typedef typename grid_type::index_iterator  index_iterator;
      typedef typename grid_type::value_type      vector3_type;
      typedef typename vector3_type::value_type  real_type;

      if(field.empty())
      {
        std::cerr << " compute_perlin_noise_force_field(): map was empty, did you forget to call create?" << std::endl;
        return;
      }


      noise::PerlinNoise<real_type> xnoise;
      noise::PerlinNoise<real_type> ynoise;
      noise::PerlinNoise<real_type> znoise;

      for( index_iterator iter = field.begin(); iter != field.end(); ++iter )
      {
        vector3_type r,v;
        OpenTissue::grid::idx2coord(iter,r);
        v(0) = xnoise(r(0),r(1),r(2));
        v(1) = ynoise(r(0),r(1),r(2));
        v(2) = znoise(r(0),r(1),r(2));
        (*iter) = v;
      }
    }

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_FORCE_UTIL_PSYS_COMPUTE_PERLIN_NOISE_FORCE_FIELD_H
#endif
