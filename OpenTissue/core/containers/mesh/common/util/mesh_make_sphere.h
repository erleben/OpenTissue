#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MAKE_SPHERE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MAKE_SPHERE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/common/util/mesh_profile_sweep.h>

namespace OpenTissue
{
  namespace mesh
  {


    /**
    * Sphere Generation.
    *
    * @param radius   The size of the radius of the sphere.
    * @param slices   The number of radial slices.
    * @param segments The number of slabs used along the z-axe.
    *
    * @return        If succesful a mesh representating
    *                a sphere otherwise null.
    */
    template<typename mesh_type, typename real_type>
    bool make_sphere(
      real_type const & radius
      , unsigned int slices
      , unsigned int segments
      , mesh_type & mesh
      )
    {
      using std::cos;
      using std::sin;

      typedef typename mesh_type::math_types                        math_types;
      typedef typename math_types::value_traits                     value_traits;
      typedef typename math_types::vector3_type                     vector3_type;
      //typedef typename math_types::real_type                        real_type;

      if(slices<3)
        throw std::invalid_argument("mesh::make_sphere(): slices was less than 3");
      if(segments<2)
        throw std::invalid_argument("mesh::make_sphere(): segments was less than 2");

      mesh.clear();

      //--- Create a profile
      std::vector<  vector3_type > profile(segments+1);

      real_type delta_angle = boost::numeric_cast<real_type>(   value_traits::pi()/segments);
      real_type angle       = delta_angle;

      profile[0] = vector3_type(value_traits::zero(),value_traits::zero(),-radius);
      
      for(unsigned int s=1;s<segments;++s)
      {
        real_type cosinus = boost::numeric_cast<real_type>( cos(angle) );
        real_type sinus   = boost::numeric_cast<real_type>( sin(angle) );

        profile[s](0) = - sinus*radius;
        profile[s](1) =  value_traits::zero();
        profile[s](2) = - cosinus * radius;
        angle += delta_angle;
      }

      profile[segments] = vector3_type(value_traits::zero(),value_traits::zero(),radius);

      return profile_sweep(profile.begin(),profile.end(),value_traits::two()*value_traits::pi(),slices,mesh);
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MAKE_SPHERE_H
#endif
