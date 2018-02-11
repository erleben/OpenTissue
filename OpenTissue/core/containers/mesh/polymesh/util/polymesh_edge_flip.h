#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_EDGE_FLIP_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_EDGE_FLIP_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_edge.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_halfedge.h>

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    *
    * Warning indices and handles of affected primitives may not be preserved!!!
    *
    *
    */
    template<typename mesh_type>
    bool edge_flip(PolyMeshHalfEdge<mesh_type> & h)
    {
      typedef typename mesh_type::face_iterator       face_iterator;
      typedef typename mesh_type::halfedge_iterator   halfedge_iterator;
      typedef typename mesh_type::edge_iterator       edge_iterator;
      typedef typename mesh_type::vertex_iterator     vertex_iterator;
      typedef typename mesh_type::face_handle         face_handle;
      typedef typename mesh_type::halfedge_handle     halfedge_handle;
      typedef typename mesh_type::edge_handle         edge_handle;
      typedef typename mesh_type::vertex_handle       vertex_handle;
      typedef typename mesh_type::face_traits         face_traits;
      typedef typename mesh_type::halfedge_traits     halfedge_traits;
      typedef typename mesh_type::edge_traits         edge_traits;
      typedef typename mesh_type::vertex_traits       vertex_traits;

      halfedge_iterator h1 = h.get_twin_iterator();
      halfedge_iterator h0 = h1->get_twin_iterator();

      edge_iterator      e = h1->get_edge_iterator();

      face_iterator     f0 = h0->get_face_iterator();
      face_iterator     f1 = h1->get_face_iterator();

      vertex_iterator   v0 = h0->get_origin_iterator();
      vertex_iterator   v1 = h1->get_next_iterator()->get_destination_iterator();
      vertex_iterator   v2 = h0->get_destination_iterator();
      vertex_iterator   v3 = h0->get_next_iterator()->get_destination_iterator();

      //std::cout << "edge_flip() :"
      //  << " v0=" << v0->get_handle().get_idx()
      //  << " v1=" << v1->get_handle().get_idx()
      //  << " v2=" << v2->get_handle().get_idx()
      //  << " v3=" << v3->get_handle().get_idx()
      //  << std::endl;
      //std::cout << "edge_flip() :"
      //  << " h0=" << h0->get_handle().get_idx()
      //  << " h1=" << h1->get_handle().get_idx()
      //  << " e =" << e->get_handle().get_idx()
      //  << std::endl;
      //std::cout << "edge_flip() :"
      //  << " f0=" << f0->get_handle().get_idx()
      //  << " f1=" << f1->get_handle().get_idx()
      //  << std::endl;


      if(is_boundary(*h0))//--- make sure that we do not flip a boundary edge
      {
        assert(!"can not flip a boundary edge");
        return false;
      }
      if(valency(*f0)!=3) //--- edge flipping is only uniquely defined for triangular face neighbors
      {
        assert(!"face 0 was not triangular");
        return false;
      }
      if(valency(*f1)!=3)
      {
        assert(!"face 1 was not triangular");
        return false;
      }
      //--- save trait data of all features that we are going to destroy
      edge_traits et      = static_cast<edge_traits> (*e);
      halfedge_traits ht0 = static_cast<halfedge_traits> (*h0);
      halfedge_traits ht1 = static_cast<halfedge_traits> (*h1);
      face_traits ft0     = static_cast<face_traits> (*f0);
      face_traits ft1     = static_cast<face_traits> (*f1);
      mesh_type * owner = h0->get_owner();
      if(owner==0)//--- unexpected error!
      {
        assert(!"Could not find owner mesh");
        return false;
      }
      //--- test if there already is an edge in the way
      halfedge_handle flipped = owner->find_halfedge_handle(v1->get_handle(),v3->get_handle());
      if(!flipped.is_null())
      {
        //assert(!"Flipped edge already exist");
        return false;
      }
      //--- remove faces
      if(!owner->remove_face(f0))
      {
        assert(!"Could not remove face 0");
        return false;
      }
      if(!owner->remove_face(f1))
      {
        assert(!"Could not remove face 1");
        return false;
      }
      //--- add new faces
      face_handle new_f0 = owner->add_face(v0->get_handle(),v1->get_handle(),v3->get_handle());
      if(new_f0.is_null())//--- unexpected error!
      {
        assert(!"Could not create new face 0");
        return false;
      }
      face_handle new_f1 = owner->add_face(v1->get_handle(),v2->get_handle(),v3->get_handle());
      if(new_f1.is_null())//--- unexpected error!
      {
        assert(!"Could not create new face 1");
        return false;
      }
      flipped = owner->find_halfedge_handle(v1->get_handle(),v3->get_handle());
      if(flipped.is_null())//--- unexpected error!
      {
        assert(!"Could not find new flipped edge");
        return false;
      }

      //--- Assign saved traits to corresponding new entities
      h0 = owner->get_halfedge_iterator(flipped);
      h1 = h0->get_twin_iterator();
      e = h0->get_edge_iterator();
      f0 = h0->get_face_iterator();
      f1 = h1->get_face_iterator();

      edge_traits     * new_et  = static_cast<edge_traits*>(& (*e));
      halfedge_traits * new_ht0 = static_cast<halfedge_traits*>(& (*h0));
      halfedge_traits * new_ht1 = static_cast<halfedge_traits*>(& (*h1));
      face_traits     * new_ft0 = static_cast<face_traits*>(& (*f0));
      face_traits     * new_ft1 = static_cast<face_traits*>(& (*f1));

      *new_et  = et;
      *new_ht0 = ht0;
      *new_ht1 = ht1;
      *new_ft0 = ft0;
      *new_ft1 = ft1;

      return true;
    }

    template<typename mesh_type>
    bool edge_flip(PolyMeshEdge<mesh_type> const & e)
    {
      typename mesh_type::halfedge_iterator  h = e.get_halfedge0_iterator();
      return edge_flip(*h);
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_EDGE_FLIP_H
#endif
