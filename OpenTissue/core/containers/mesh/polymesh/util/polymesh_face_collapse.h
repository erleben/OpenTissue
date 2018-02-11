#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_FACE_COLLAPSE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_FACE_COLLAPSE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_face_center.h>

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    * Collapse Face into a Vertex.
    *
    * Warning indices and handles of affected primitives may not be preserved!!!
    *
    *
    *  Work in Progress, does not quite work as it should!!!
    *
    * @param f
    *
    * @return
    */
    template<typename mesh_type>
    bool face_collapse(PolyMeshFace<mesh_type> & f)
    {
      typedef typename mesh_type::face_halfedge_circulator        face_halfedge_circulator;
      typedef typename mesh_type::halfedge_iterator               halfedge_iterator;
      typedef typename mesh_type::halfedge_type                   halfedge_type;
      typedef typename mesh_type::vertex_handle                   vertex_handle;
      typedef typename mesh_type::face_handle                     face_handle;

      typedef typename mesh_type::math_types                      math_types;
      typedef typename math_types::vector3_type                   vector3_type;

      mesh_type * owner = f.get_owner();
      if(owner==0)
      {
        assert(!"face_collapse(): No mesh owner!");
        return false;
      }
      int i;
      int n = valency(f);

      typedef std::vector<vertex_handle> boundary_container;

      vector3_type center(0,0,0);
      mesh::compute_face_center(f,center);
      vertex_handle collapsed = owner->add_vertex(center);

      std::list<face_handle> neighbors;
      std::vector<boundary_container> boundaries(n);
      {
        face_halfedge_circulator cur(f);
        for(i=0; i<n; ++i,++cur)
        {
          halfedge_iterator twin = cur->get_twin_iterator();
          halfedge_iterator first = twin->get_next_iterator();
          halfedge_iterator last  = twin->get_prev_iterator();
          if(!twin->get_face_handle().is_null())
          {
            neighbors.push_back( twin->get_face_handle() );
            halfedge_type * loop = &(*first);
            halfedge_type * stop = &(*last);
            do
            {
              boundaries[i].push_back( loop->get_destination_handle() );
              loop = &(*(loop->get_next_iterator()));
            }while(loop != stop );
            boundaries[i].push_back( collapsed );
          }
        }
      }

      for(typename std::list<face_handle>::iterator fi = neighbors.begin(); fi != neighbors.end(); ++fi)
        owner->remove_face( *fi );

      for(i=0;i<n;++i)
      {
        if(boundaries[i].size()>2)
          owner->add_face(boundaries[i].begin(),boundaries[i].end());
      }

      return true;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_FACE_COLLAPSE_H
#endif
