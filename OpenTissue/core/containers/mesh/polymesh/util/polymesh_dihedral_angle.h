#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_DIHEDRAL_ANGLE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_DIHEDRAL_ANGLE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_halfedge.h>

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    */
    template<typename mesh_type,typename real_type>
    void compute_dihedral_angle(PolyMeshHalfEdge<mesh_type> const & e, real_type & radian)
    {
      typedef typename mesh_type::math_types       math_types;
      typedef typename math_types::vector3_type    vector3_type;
      typedef typename mesh_type::vertex_iterator  vertex_iterator;
      typedef typename mesh_type::face_iterator    face_iterator;
      static real_type threshold = math::working_precision<real_type>();

      if( e.get_face_handle().is_null() || e.get_twin_iterator()->get_face_handle().is_null() )
      {
        std::cout << "compute_dihedral_angle(): boundary edge encountered!" << std::endl;
        radian = 0;
        return;
      }

      face_iterator face      = e.get_face_iterator();
      face_iterator twin_face = e.get_twin_iterator()->get_face_iterator();

      vector3_type n1,n2;
      compute_face_normal(*face, n1);
      compute_face_normal(*twin_face, n2);

      real_type denom = length(n1)*length(n2);
      if ( denom < threshold)
      {
        radian = 0;
        return;
      }
      real_type cos_angle = n1*n2 / denom;
      cos_angle = (cos_angle>1.0)?1.0: ( (cos_angle<-1.0)?-1.0:cos_angle  );

      vertex_iterator d = e.get_destination_iterator();
      vertex_iterator o = e.get_origin_iterator();
      vector3_type u    = d->m_coord - o->m_coord;
      real_type sin_angle = (n1 % n2) * u;
      sin_angle = (sin_angle>1.0)?1.0: ( (sin_angle<-1.0)?-1.0:sin_angle  );
      radian = sin_angle >= 0 ? acos(cos_angle) : -acos(cos_angle);
    }

    template<typename mesh_type,typename real_type>
    void compute_dihedral_angle(PolyMeshEdge<mesh_type> const & e, real_type & radian)
    {
      compute_dihedral_angle( *(e.get_halfedge0_iterator()), radian );
    }


  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_DIHEDRAL_ANGLE_H
#endif

