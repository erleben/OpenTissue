#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MAKE_DISK_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MAKE_DISK_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/common/util/mesh_profile_sweep.h>
#include <OpenTissue/core/math/math_constants.h>

#include <vector>

namespace OpenTissue
{
  namespace mesh
  {


    /**
    * Disk Generation.
    *
    * @param radius1  Specifies the circular size of the disk (maximum radius would be the sum of the two radie).
    * @param radius2  Specifies the curveness (flatness) of the disk.
    * @param slices   The number of radial slices.
    * @param segments The number of slabs used along the z-axe.
    *
    * @return        If succesful a mesh representating
    *                a sphere otherwise null.
    */
    template<typename mesh_type, typename real_type>
    bool make_disk(
      real_type const & radius1
      , real_type const & radius2
      , unsigned int slices
      , unsigned int segments
      , mesh_type & mesh
      )
    {
      typedef typename mesh_type::math_types                        math_types;
      typedef typename math_types::value_traits                     value_traits;
      typedef typename math_types::vector3_type                     vector3_type;
      //typedef typename math_types::real_type                        real_type;

      assert(slices>=3);
      assert(segments>=2);

      //--- Create a profile
      std::vector<vector3_type> profile(segments+3);
      real_type delta_angle = static_cast<real_type>(math::detail::pi<real_type>()/segments);
      real_type angle       = delta_angle;

      profile[0] = vector3_type(0,0,-radius2);
      profile[1] = vector3_type(-radius1,0,-radius2);
      for(unsigned int s=2;s<=(segments);++s)
      {
        real_type cosinus = static_cast<real_type>(std::cos(angle));
        real_type sinus = static_cast<real_type>(std::sin(angle));

        profile[s](0) = - sinus*radius2 - radius1;
        profile[s](1) =  0;
        profile[s](2) = - cosinus * radius2 ;

        angle += delta_angle;
      }
      profile[segments+1] = vector3_type(-radius1,0,radius2);
      profile[segments+2] = vector3_type(0,0,radius2);
      return profile_sweep(profile.begin(),profile.end(),2.0*math::detail::pi<real_type>(),slices,mesh);
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MAKE_DISK_H
#endif
