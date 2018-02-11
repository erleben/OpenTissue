#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_POINT_INSIDE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_POINT_INSIDE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_boundary.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_valency.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_edge_direction.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_convex_boundary.h>

#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_face_maximum_coord.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_face_minimum_coord.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_face_plane.h>

#include <OpenTissue/core/geometry/geometry_plane.h>
#include <OpenTissue/core/geometry/geometry_ray.h>
#include <OpenTissue/collision/collision_ray_aabb.h>


namespace OpenTissue
{
  namespace polymesh
  {

    /**
     * This function implements an odd-even parity check using a ray shoot
     * along a pre-defined direction.
     *
     * This function has one major drawback! If a ray hits precisely on
     * an edge, then one will compute two intersections of the polygonal surface (one
     * hit from each face sharing the edge) and not one intersection as expected.
     *
     * This is not easily fixed!
     *
     */
    template<typename mesh_type,typename vector3_type>
    inline bool is_point_inside(mesh_type const & mesh, vector3_type const & p)
    {
      typedef typename mesh_type::math_types                 math_types;
      typedef typename math_types::real_type                 real_type;
      typedef geometry::Plane<math_types>                    plane_type;


      vector3_type u;         //--- Edge direction vector
      vector3_type r = unit( vector3_type(0.3,0.6, 1.0) );  //--- Ray direction
      plane_type plane;       //--- Face plane

      int intersections = 0;

      typename mesh_type::const_face_iterator begin = mesh.face_begin();
      typename mesh_type::const_face_iterator end   = mesh.face_end();
      typename mesh_type::const_face_iterator f     = begin;
      for(;f!=end;++f)
      {
        assert(is_convex_boundary( *f ) || !"only works on convex faces");

        vector3_type max_coord;
        mesh::compute_face_maximum_coord(*f,max_coord);
        if(p(0) > max_coord(0)) //--- We allways use a ray pointing in the positive cone!
          continue;

        vector3_type min_coord;
        mesh::compute_face_minimum_coord(*f,min_coord);
        if(!OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord))
          continue;

        mesh::compute_face_plane( *f, plane );
        real_type  d  = plane.signed_distance(p);
        real_type  nr = plane.n() * r;

        //--- special case not handled: Ray in face plane => nr==0 && d==0
        //if(nr==0 && d == 0)
        //{
        //   ... todo ....
        //}

        if(nr==0 && d != 0)
          continue;

        if(nr<0 && d < 0)
          continue;

        if(nr>0 && d > 0)
          continue;

        real_type    t  =  -d / nr;
        vector3_type pp =   p + r*t;

        bool ray_inside_face = true;
        typename mesh_type::face_halfedge_circulator h(*f),hend;
        for(;h!=hend;++h)
        {
          compute_edge_direction( *h, u);
          vector3_type diff = pp - h->get_origin_iterator()->m_coord;
          if( plane.n() * cross( u , diff ) < 0)
          {
            ray_inside_face = false;
            break;
          }
        }
        if(ray_inside_face)
          ++intersections;
      }
      return ((intersections%2)==1);
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_POINT_INSIDE_H
#endif
