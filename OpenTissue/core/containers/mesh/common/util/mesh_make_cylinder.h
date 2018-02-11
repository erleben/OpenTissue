#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MAKE_CYLINDER_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MAKE_CYLINDER_H
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
    * Cylinder Generation.
    * This method generates a cylinder aligned with the z-axe and
    * with origo at the center of the cylinder.
    *
    * @param radius  The size of the radius of the cylinder.
    * @param height  The total height of the cylinder.
    * @param slices  The number of slices used to represent
    *                the cylinder.
    *
    * @return        If succesful a mesh representating
    *                a cylinder otherwise null.
    */
    template<typename mesh_type, typename real_type>
    bool make_cylinder(
      real_type const & radius
      , real_type const & height
      , unsigned int slices
      , mesh_type & mesh
      )
    {
      typedef typename mesh_type::math_types                        math_types;
      typedef typename math_types::value_traits                     value_traits;
      typedef typename math_types::vector3_type                     vector3_type;
      //typedef typename math_types::real_type                        real_type;

      assert(slices>=3);

      std::vector<vector3_type> profile(4);
      real_type h= height/2.;
      profile[0] = vector3_type(0,0,-h);
      profile[1] = vector3_type(radius,0,-h);
      profile[2] = vector3_type(radius,0,h);
      profile[3] = vector3_type(0,0,h);

      return profile_sweep(profile.begin(),profile.end(),2.0*math::detail::pi<real_type>(),slices,mesh);
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MAKE_CYLINDER_H
#endif
