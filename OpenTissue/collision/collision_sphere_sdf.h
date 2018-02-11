#ifndef OPENTISSUE_COLLISION_COLLISION_SPHERE_SDF_H
#define OPENTISSUE_COLLISION_COLLISION_SPHERE_SDF_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/grid/util/grid_is_point_inside.h>

namespace OpenTissue
{
  namespace collision
  {

    /**
    * Test Sphere Against Signed Distance Map.
    *
    * @param wcsS       The world location of the sphere geometry.
    * @param wcsG       The world location of the signed distance field geometry.
    * @param sphere     The geometry of object A (the sphere geometry).
    * @param geometry   The geometry of object B ( the signed distance field geometry).
    * @param contacts   Upon return holds all the contact points between the two object. By convention contact normals always point from sdf object towards sphere object.
    * @param envelope   The size of the collision envelope, default value is 0.01. Whenever objects are within this distance then contact points will be generated.
    *
    * @return           If a collision is detected then the return value is true otherwise it is false.
    */
    template<typename coordsys_type,typename sdf_geometry_type,typename sphere_type,typename contact_point_container>
    bool sphere_sdf(
      coordsys_type & wcsS
      , sphere_type const & sphere
      , coordsys_type & wcsG
      , sdf_geometry_type const & geometry
      , contact_point_container & contacts
      , double envelope = 0.01
      )
    {
      typedef typename contact_point_container::value_type   contact_point_type;
      typedef typename coordsys_type::vector3_type           vector3_type;
      typedef typename vector3_type::value_type              real_type;

      contacts.clear();

      //--- Transform sphere center into model frame of signed distance map
      coordsys_type StoG;
      StoG = model_update(wcsS,wcsG);
      vector3_type c,p,n;
      c = sphere.center();
      StoG.xform_point( c );

      if ( !OpenTissue::grid::is_point_inside( geometry.m_phi, c) )
        return false;

      real_type distance = OpenTissue::grid::value_at_point( geometry.m_phi, c);
      if ( distance == geometry.m_phi.unused() )
        return false;

      distance -= sphere.radius();
      real_type tst = boost::numeric_cast<real_type>( envelope );
      if(distance > tst)
        return false;

      vector3_type gradient = OpenTissue::grid::gradient_at_point( geometry.m_phi, c );
      if ( gradient(0) == geometry.m_phi.unused() )
        return false;

      gradient = unit(gradient);
      c = c - gradient*sphere.radius();
      wcsG.xform_vector( gradient );
      wcsG.xform_point( c );

      contact_point_type cp;
      cp.m_p = c;
      cp.m_n = gradient;
      cp.m_distance = distance;

      contacts.push_back( cp );

      return true;
    }

  } //End of namespace collision
} // namespace OpenTissue

// OPENTISSUE_COLLISION_COLLISION_SPHERE_SDF_H
#endif
