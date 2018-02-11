#ifndef OPENTISSUE_DYNAMICS_PSYS_FORCE_UTIL_PSYS_COMPUTE_RANDOM_FORCE_FIELD_H
#define OPENTISSUE_DYNAMICS_PSYS_FORCE_UTIL_PSYS_COMPUTE_RANDOM_FORCE_FIELD_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/cast.hpp> //--- needed for boost::numeric_cast
#include <cassert>

namespace OpenTissue
{
  namespace psys
  {

    template<typename grid_type>
      void compute_random_force_field( grid_type & field, double magnitude )
    {
      assert(magnitude>0 || !"compute_random_force_field(): magnitude was non-positive");

      typedef typename grid_type::iterator        iterator;
      typedef typename grid_type::value_type      vector3_type;
      typedef typename vector3_type::value_type  real_type;


      if(field.empty())
      {
        std::cerr << " compute_random_force_field(): map was empty, did you forget to call create?" << std::endl;
        return;
      }

      real_type  higher =  boost::numeric_cast <real_type>(magnitude);
      real_type  lower  = - higher;

      for( iterator iter = field.begin(); iter != field.end(); ++iter )
        random((*iter),lower,higher);
    }

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_FORCE_UTIL_PSYS_COMPUTE_RANDOM_FORCE_FIELD_H
#endif
