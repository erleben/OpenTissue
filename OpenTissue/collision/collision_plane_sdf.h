#ifndef OPENTISSUE_COLLISION_COLLISION_PLANE_SDF_H
#define OPENTISSUE_COLLISION_COLLISION_PLANE_SDF_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace collision
  {

    /**
    * Test Signed Distance Map against plane.
    *
    * The current implementation simply test every sample point against the
    * specified plane. This is very naive and not very efficient. It would be
    * much faster to test the BVH of the signed distance field geometry against
    * the plane geometry. This is left for future optimization.
    *
    * @param wcsP       The world location of the plane geometry.
    * @param plane      The object A geometry ( a plane).
    * @param wcsG       The world location of the signed distance field geometry.
    * @param geometry   The object B geometry (signed distance field). 
    * @param contacts   Upon return holds all the contact points between the two object. By convention contact normals alway point from plane object towards sdf object.
    * @param envelope   The size of the collision envelope, default value is 0.01. Whenever objects are within this distance then contact points will be generated.
    *
    * @return           If a collision is detected then the return value is true otherwise it is false.
    */
    template<typename coordsys_type,typename plane_type,typename sdf_geometry_type,typename contact_point_container>
    bool plane_sdf(
      coordsys_type     const & wcsP
      , plane_type        const & plane
      , coordsys_type     const & wcsG
      , sdf_geometry_type const & geometry
      , contact_point_container & contacts
      , double envelope = 0.01
      )
    {
      typedef typename contact_point_container::value_type         contact_point_type;
      typedef typename coordsys_type::vector3_type                 vector3_type;
      typedef typename vector3_type::value_type                    real_type;
      typedef typename sdf_geometry_type::const_point_iterator     const_point_iterator;

      contacts.clear();

      vector3_type c,n,n_wcs;
      coordsys_type GtoP;
      GtoP = model_update(wcsG,wcsP);

      n = plane.n();
      n_wcs = n;
      wcsP.xform_vector(n_wcs);

      bool collision = false;

      const_point_iterator end = geometry.m_sampling.end();
      const_point_iterator p   = geometry.m_sampling.begin();

      real_type tst = boost::numeric_cast<real_type>( envelope ); 

      for (;p!=end;++p)
      {
        c = (*p);
        GtoP.xform_point(c);

        real_type dist = plane.signed_distance(c);

        if(dist < tst)
        {
          c = (*p);
          wcsG.xform_point(c);

          contact_point_type cp;
          cp.m_p = c;
          cp.m_n = n_wcs;
          cp.m_distance = dist;
          contacts.push_back(cp);

          collision = true;
        }
      }
      return collision;
    }

  } //End of namespace collision

} // namespace OpenTissue

// OPENTISSUE_COLLISION_COLLISION_PLANE_SDF_H
#endif
